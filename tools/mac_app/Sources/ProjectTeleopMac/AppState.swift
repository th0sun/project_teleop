import Foundation
import Combine

enum LaunchMode: String, CaseIterable, Identifiable {
    case mockSim = "mock-sim"
    case realSim = "real-sim"
    case mockUnity = "mock-unity"
    case realUnity = "real-unity"

    var id: String { rawValue }
    var label: String {
        switch self {
        case .mockSim: return "Mock + Mac Simulator"
        case .realSim: return "Real MG400 + Mac Simulator"
        case .mockUnity: return "Mock + Unity"
        case .realUnity: return "Real MG400 + Unity"
        }
    }
    var systemImage: String {
        switch self {
        case .mockSim, .mockUnity: return "cube.transparent"
        case .realSim, .realUnity: return "bolt.fill"
        }
    }
}

enum ContainerStatus: String {
    case running, stopped, unknown, working
    var label: String {
        switch self {
        case .running: return "Running"
        case .stopped: return "Stopped"
        case .working: return "Working…"
        case .unknown: return "Unknown"
        }
    }
}

enum ConsolePane: String, CaseIterable, Identifiable {
    case status, teleop, endpoint, simulator, unity, rviz, mock, container, build

    var id: String { rawValue }

    var label: String {
        switch self {
        case .status: return "Status"
        case .teleop: return "Teleop"
        case .endpoint: return "Endpoint"
        case .simulator: return "Simulator"
        case .unity: return "Unity"
        case .rviz: return "RViz"
        case .mock: return "Mock"
        case .container: return "Container"
        case .build: return "Build"
        }
    }

    var launcherArgument: String { rawValue }
}

enum RobotDashboardCommand: String, CaseIterable, Identifiable {
    case enable = "enable"
    case disable = "disable"
    case clearError = "clear-error"
    case reset = "reset"
    case emergencyStop = "emergency-stop"

    var id: String { rawValue }

    var label: String {
        switch self {
        case .enable: return "Enable"
        case .disable: return "Disable"
        case .clearError: return "Clear Error"
        case .reset: return "Reset"
        case .emergencyStop: return "E-Stop"
        }
    }

    var dashboardCommand: String {
        switch self {
        case .enable: return "EnableRobot()"
        case .disable: return "DisableRobot()"
        case .clearError: return "ClearError()"
        case .reset: return "ResetRobot()"
        case .emergencyStop: return "EmergencyStop()"
        }
    }

    var systemImage: String {
        switch self {
        case .enable: return "power.circle.fill"
        case .disable: return "power.circle"
        case .clearError: return "checkmark.shield.fill"
        case .reset: return "arrow.counterclockwise.circle.fill"
        case .emergencyStop: return "exclamationmark.octagon.fill"
        }
    }
}

@MainActor
final class AppState: ObservableObject {
    // settings
    @Published var repoPath: String {
        didSet { UserDefaults.standard.set(repoPath, forKey: "repoPath") }
    }
    @Published var selectedMode: LaunchMode {
        didSet { UserDefaults.standard.set(selectedMode.rawValue, forKey: "selectedMode") }
    }

    // launcher
    @Published var containerStatus: ContainerStatus = .unknown
    @Published var launcherLog: [String] = []
    @Published var selectedConsolePane: ConsolePane = .status
    @Published var serviceLog: [String] = []
    @Published var loadingServiceLog = false
    @Published var robotCommandBusy = false
    @Published var lastRobotCommandStatus = "no robot command sent"

    // robot live state
    @Published var jointPositions: [Double] = [0, 0, 0, 0]
    @Published var jointPositionsDeg: [Double] = [0, 0, 0, 0]
    @Published var jointNames: [String] = ["joint1", "joint2", "joint3", "joint4"]
    @Published var unityJointPositions: [Double]?
    @Published var unityJointPositionsDeg: [Double]?
    @Published var unityJointNames: [String] = []
    @Published var robotMode: Int = 0
    @Published var errorStatus: Int = 0
    @Published var suction: Bool = false
    @Published var light: Bool = false
    @Published var lastJointUpdate: Date?
    @Published var lastUnityJointUpdate: Date?
    @Published var bridgeConnected: Bool = false

    let scene = MG400SceneController()

    private let docker = DockerService()
    private var bridgeTask: Task<Void, Never>?
    private var statusPoll: Task<Void, Never>?

    init() {
        let defaults = UserDefaults.standard
        let fallback = "/Users/thesun/the_core/Robotics_and_PLC/project_teleop_ws/project_teleop"
        self.repoPath = defaults.string(forKey: "repoPath") ?? fallback
        self.selectedMode = LaunchMode(rawValue: defaults.string(forKey: "selectedMode") ?? "") ?? .mockSim
        startStatusPolling()
        Task { @MainActor [weak self] in
            guard let self else { return }
            self.scene.configure(repoPath: self.repoPath)
        }
    }

    var launcherURL: URL {
        URL(fileURLWithPath: repoPath).appendingPathComponent("tools/mac_app/launcher.sh")
    }

    var sessionsDir: URL {
        URL(fileURLWithPath: repoPath).appendingPathComponent("logs/teleop_sessions")
    }

    var currentConsoleLines: [String] {
        selectedConsolePane == .status ? launcherLog : serviceLog
    }

    // MARK: - Launcher

    func appendLauncher(_ line: String) {
        guard !line.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty else { return }
        launcherLog.append(line)
        if launcherLog.count > 400 {
            launcherLog.removeFirst(launcherLog.count - 400)
        }
    }

    func start() {
        let mode = selectedMode.rawValue
        selectedConsolePane = .status
        serviceLog.removeAll()
        launcherLog.removeAll()
        appendLauncher("$ launcher.sh start \(mode)")
        containerStatus = .working
        stopBridge()
        Task {
            do {
                let stream = try docker.spawn(
                    launcher: launcherURL,
                    arguments: ["start", mode]
                )
                for await line in stream {
                    appendLauncher(line)
                }
                await refreshStatus()
                await refreshSelectedConsole()
                if containerStatus == .running { startBridge() }
            } catch {
                appendLauncher("error: \(error.localizedDescription)")
                containerStatus = .unknown
            }
        }
    }

    func stop() {
        selectedConsolePane = .status
        serviceLog.removeAll()
        appendLauncher("$ launcher.sh stop")
        containerStatus = .working
        stopBridge()
        Task {
            do {
                let stream = try docker.spawn(launcher: launcherURL, arguments: ["stop"])
                for await line in stream { appendLauncher(line) }
                await refreshStatus()
            } catch {
                appendLauncher("error: \(error.localizedDescription)")
            }
        }
    }

    func refreshSelectedConsole() async {
        if selectedConsolePane == .status {
            await refreshStatus()
            return
        }
        await refreshServiceLog()
    }

    func refreshServiceLog() async {
        guard selectedConsolePane != .status else { return }
        loadingServiceLog = true
        defer { loadingServiceLog = false }
        do {
            let result = try await docker.runOnce(
                launcher: launcherURL,
                arguments: ["logs", selectedConsolePane.launcherArgument]
            )
            serviceLog = result
                .split(whereSeparator: \.isNewline)
                .map(String.init)
            if serviceLog.isEmpty {
                serviceLog = ["no \(selectedConsolePane.rawValue) log output"]
            }
        } catch {
            serviceLog = ["error: \(error.localizedDescription)"]
        }
    }

    func clearSelectedConsole() {
        if selectedConsolePane == .status {
            launcherLog.removeAll()
        } else {
            serviceLog.removeAll()
        }
    }

    func sendRobotCommand(_ command: RobotDashboardCommand) async {
        robotCommandBusy = true
        lastRobotCommandStatus = "sending \(command.dashboardCommand)"
        appendLauncher("$ launcher.sh robot \(command.rawValue)")
        defer { robotCommandBusy = false }
        do {
            let result = try await docker.runOnce(
                launcher: launcherURL,
                arguments: ["robot", command.rawValue]
            )
            let lines = result
                .split(whereSeparator: \.isNewline)
                .map(String.init)
            let published = lines.last(where: { $0.contains("published /robot/dashboard_cmd") })
            lastRobotCommandStatus = published ?? lines.last ?? "sent \(command.dashboardCommand)"
            if selectedConsolePane == .status {
                appendLauncher(lastRobotCommandStatus)
            }
        } catch {
            lastRobotCommandStatus = "robot command failed: \(error.localizedDescription)"
            appendLauncher(lastRobotCommandStatus)
        }
    }

    func refreshStatus() async {
        do {
            let result = try await docker.runOnce(
                launcher: launcherURL,
                arguments: ["status"]
            )
            let trimmed = result.trimmingCharacters(in: .whitespacesAndNewlines)
            containerStatus = trimmed == "running" ? .running : .stopped
            if containerStatus == .running, !bridgeConnected {
                startBridge()
            } else if containerStatus == .stopped {
                bridgeConnected = false
                stopBridge()
            }
        } catch {
            containerStatus = .unknown
        }
    }

    private func startStatusPolling() {
        statusPoll?.cancel()
        statusPoll = Task { [weak self] in
            while !Task.isCancelled {
                await self?.refreshStatus()
                try? await Task.sleep(nanoseconds: 4_000_000_000)
            }
        }
    }

    // MARK: - Bridge

    func startBridge() {
        guard bridgeTask == nil else { return }
        bridgeConnected = false
        bridgeTask = Task { [weak self] in
            guard let self else { return }
            do {
                let stream = try self.docker.spawn(
                    launcher: self.launcherURL,
                    arguments: ["bridge"]
                )
                for await line in stream {
                    self.handleBridgeLine(line)
                }
            } catch {
                self.appendLauncher("bridge error: \(error.localizedDescription)")
            }
            self.bridgeConnected = false
            self.bridgeTask = nil
        }
    }

    func stopBridge() {
        bridgeTask?.cancel()
        bridgeTask = nil
        docker.terminateBridge()
        bridgeConnected = false
    }

    private func handleBridgeLine(_ line: String) {
        guard let data = line.data(using: .utf8) else { return }
        guard let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return }
        guard let kind = obj["t"] as? String else { return }
        switch kind {
        case "ready":
            bridgeConnected = true
        case "joint_states":
            if let pos = obj["position"] as? [Double] { jointPositions = pos }
            if let pos = obj["position_deg"] as? [Double] {
                jointPositionsDeg = pos
                scene.updateActual(pos)
            }
            if let names = obj["name"] as? [String] { jointNames = names }
            lastJointUpdate = Date()
            bridgeConnected = true
        case "unity_joint_cmd":
            if let pos = obj["position"] as? [Double] { unityJointPositions = pos }
            if let pos = obj["position_deg"] as? [Double] {
                unityJointPositionsDeg = pos
                scene.updateUnity(pos)
                scene.markUnityFresh()
            }
            if let names = obj["name"] as? [String] { unityJointNames = names }
            lastUnityJointUpdate = Date()
            bridgeConnected = true
        case "robot_mode":
            if let v = obj["value"] as? Int { robotMode = v }
        case "error_status":
            if let v = obj["value"] as? Int { errorStatus = v }
        case "suction":
            if let v = obj["value"] as? Bool { suction = v }
        case "light":
            if let v = obj["value"] as? Bool { light = v }
        default:
            break
        }
    }
}
