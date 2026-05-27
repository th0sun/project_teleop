import SwiftUI

struct MonitorView: View {
    @EnvironmentObject var state: AppState

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                header
                statusGrid
                controlsCard
                jointsCard
                ioCard
            }
            .padding(24)
        }
    }

    private var header: some View {
        HStack(alignment: .firstTextBaseline) {
            VStack(alignment: .leading, spacing: 4) {
                Text("Robot Monitor").font(.largeTitle.weight(.semibold))
                Text("Live state from `/joint_states`, `/mg400/robot_mode`, `/mg400/error_status`.")
                    .foregroundStyle(.secondary)
            }
            Spacer()
            BridgeBadge()
        }
    }

    private var statusGrid: some View {
        let cols = [GridItem(.adaptive(minimum: 220), spacing: 12)]
        return LazyVGrid(columns: cols, spacing: 12) {
            StatusTile(title: "Robot Mode",
                       value: "\(state.robotMode)",
                       caption: robotModeText,
                       tint: robotModeColor,
                       icon: "gauge.with.dots.needle.bottom.50percent")
            StatusTile(title: "Error",
                       value: "\(state.errorStatus)",
                       caption: state.errorStatus == 0 ? "OK" : "Controller error",
                       tint: state.errorStatus == 0 ? .green : .red,
                       icon: state.errorStatus == 0 ? "checkmark.shield.fill" : "exclamationmark.triangle.fill")
            StatusTile(title: "Bridge",
                       value: state.bridgeConnected ? "live" : "offline",
                       caption: lastUpdateText,
                       tint: state.bridgeConnected ? .green : .gray,
                       icon: "antenna.radiowaves.left.and.right")
            StatusTile(title: "Container",
                       value: state.containerStatus.label,
                       caption: state.selectedMode.rawValue,
                       tint: state.containerStatus == .running ? .green : .gray,
                       icon: "shippingbox.fill")
        }
    }

    private var controlsCard: some View {
        Card(title: "Robot controls", systemImage: "switch.2") {
            let cols = [GridItem(.adaptive(minimum: 150), spacing: 10)]
            LazyVGrid(columns: cols, spacing: 10) {
                ForEach(RobotDashboardCommand.allCases) { command in
                    RobotCommandButton(
                        command: command,
                        tint: commandTint(command),
                        disabled: state.containerStatus != .running || state.robotCommandBusy
                    ) {
                        Task { await state.sendRobotCommand(command) }
                    }
                }
            }
            Text(state.lastRobotCommandStatus)
                .font(.caption.monospaced())
                .foregroundStyle(.secondary)
                .lineLimit(3)
        }
    }

    private var jointsCard: some View {
        Card(title: "Joint angles", systemImage: "figure.walk.motion") {
            let cols = [GridItem(.flexible()), GridItem(.flexible()),
                        GridItem(.flexible()), GridItem(.flexible())]
            LazyVGrid(columns: cols, spacing: 12) {
                ForEach(0..<min(4, state.jointPositions.count), id: \.self) { idx in
                    JointTile(
                        index: idx + 1,
                        name: idx < state.jointNames.count ? state.jointNames[idx] : "j\(idx + 1)",
                        radians: state.jointPositions[idx]
                    )
                }
            }
        }
    }

    private func commandTint(_ command: RobotDashboardCommand) -> Color {
        switch command {
        case .enable: return .green
        case .disable: return .orange
        case .clearError: return .yellow
        case .reset: return .blue
        case .emergencyStop: return .red
        }
    }

    private var ioCard: some View {
        Card(title: "Outputs", systemImage: "switch.2") {
            HStack(spacing: 16) {
                IOTile(label: "Suction", on: state.suction, onColor: .blue, icon: "drop.fill")
                IOTile(label: "Light",   on: state.light,   onColor: .yellow, icon: "lightbulb.fill")
                Spacer()
            }
        }
    }

    private var robotModeText: String {
        switch state.robotMode {
        case 0: return "Idle"
        case 1: return "INIT"
        case 4: return "DISABLED"
        case 5: return "ENABLE"
        case 6: return "DRAG"
        case 7: return "RUN"
        case 9: return "ERROR"
        case 11: return "COLLISION"
        default: return "\(state.robotMode)"
        }
    }
    private var robotModeColor: Color {
        switch state.robotMode {
        case 1: return .blue
        case 5, 7: return .green
        case 4, 6: return .orange
        case 9, 11: return .red
        default: return .gray
        }
    }
    private var lastUpdateText: String {
        guard let t = state.lastJointUpdate else { return "no data" }
        let ago = Date().timeIntervalSince(t)
        return String(format: "updated %.1fs ago", ago)
    }
}

struct RobotCommandButton: View {
    let command: RobotDashboardCommand
    let tint: Color
    let disabled: Bool
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            Label(command.label, systemImage: command.systemImage)
                .font(.subheadline.weight(.semibold))
                .lineLimit(1)
                .frame(maxWidth: .infinity)
                .padding(.vertical, 10)
                .padding(.horizontal, 12)
                .foregroundStyle(.white)
                .background(
                    RoundedRectangle(cornerRadius: 8)
                        .fill(disabled ? Color.gray.opacity(0.45) : tint)
                )
        }
        .buttonStyle(.plain)
        .disabled(disabled)
        .help(command.dashboardCommand)
    }
}

struct BridgeBadge: View {
    @EnvironmentObject var state: AppState
    var body: some View {
        HStack(spacing: 8) {
            Image(systemName: state.bridgeConnected
                  ? "dot.radiowaves.left.and.right"
                  : "wifi.slash")
                .foregroundStyle(state.bridgeConnected ? .green : .secondary)
            Text(state.bridgeConnected ? "Bridge live" : "Bridge offline")
                .font(.subheadline.weight(.medium))
        }
        .padding(.horizontal, 12).padding(.vertical, 6)
        .background(Capsule().fill(Color(nsColor: .controlBackgroundColor)))
        .overlay(Capsule().strokeBorder(Color.secondary.opacity(0.2)))
    }
}

struct StatusTile: View {
    let title: String
    let value: String
    let caption: String
    let tint: Color
    let icon: String

    var body: some View {
        HStack(alignment: .top, spacing: 12) {
            Image(systemName: icon)
                .font(.title)
                .foregroundStyle(tint)
                .frame(width: 36)
            VStack(alignment: .leading, spacing: 2) {
                Text(title).font(.caption).foregroundStyle(.secondary).textCase(.uppercase)
                Text(value).font(.title2.weight(.semibold))
                Text(caption).font(.caption).foregroundStyle(.secondary)
            }
            Spacer()
        }
        .padding(14)
        .background(RoundedRectangle(cornerRadius: 12).fill(Color(nsColor: .controlBackgroundColor)))
        .overlay(RoundedRectangle(cornerRadius: 12).strokeBorder(Color.secondary.opacity(0.15)))
    }
}

struct JointTile: View {
    let index: Int
    let name: String
    let radians: Double

    var body: some View {
        VStack(spacing: 6) {
            Text("J\(index)")
                .font(.caption.weight(.medium))
                .foregroundStyle(.secondary)
            Text(String(format: "%.2f°", radians * 180.0 / .pi))
                .font(.system(.title2, design: .rounded).weight(.semibold))
                .monospacedDigit()
            Text(name)
                .font(.caption2)
                .foregroundStyle(.secondary)
                .lineLimit(1)
        }
        .frame(maxWidth: .infinity)
        .padding(12)
        .background(RoundedRectangle(cornerRadius: 10).fill(Color.accentColor.opacity(0.08)))
    }
}

struct IOTile: View {
    let label: String
    let on: Bool
    let onColor: Color
    let icon: String
    var body: some View {
        HStack(spacing: 10) {
            Image(systemName: icon)
                .foregroundStyle(on ? onColor : .secondary)
            VStack(alignment: .leading, spacing: 1) {
                Text(label).font(.subheadline.weight(.medium))
                Text(on ? "ON" : "off")
                    .font(.caption)
                    .foregroundStyle(on ? onColor : .secondary)
            }
        }
        .padding(.horizontal, 14).padding(.vertical, 10)
        .background(RoundedRectangle(cornerRadius: 10)
            .fill(on ? onColor.opacity(0.15) : Color.secondary.opacity(0.08)))
    }
}

struct Card<Content: View>: View {
    let title: String
    let systemImage: String
    @ViewBuilder var content: () -> Content
    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Label(title, systemImage: systemImage).font(.headline)
            content()
        }
        .padding(16)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(RoundedRectangle(cornerRadius: 14).fill(Color(nsColor: .controlBackgroundColor)))
        .overlay(RoundedRectangle(cornerRadius: 14).strokeBorder(Color.secondary.opacity(0.15)))
    }
}
