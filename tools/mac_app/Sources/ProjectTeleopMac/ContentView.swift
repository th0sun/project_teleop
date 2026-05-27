import SwiftUI

enum AppTab: String, CaseIterable, Identifiable {
    case launcher, monitor, view3d, logs
    var id: String { rawValue }
    var label: String {
        switch self {
        case .launcher: return "Launcher"
        case .monitor:  return "Monitor"
        case .view3d:   return "3D View"
        case .logs:     return "Sessions"
        }
    }
    var icon: String {
        switch self {
        case .launcher: return "play.rectangle.fill"
        case .monitor:  return "waveform.path.ecg"
        case .view3d:   return "cube.fill"
        case .logs:     return "doc.text.magnifyingglass"
        }
    }
}

struct ContentView: View {
    @EnvironmentObject var state: AppState
    @State private var selection: AppTab = .launcher

    var body: some View {
        NavigationSplitView {
            List(selection: $selection) {
                Section("Project Teleop") {
                    ForEach(AppTab.allCases) { tab in
                        Label(tab.label, systemImage: tab.icon).tag(tab)
                    }
                }
            }
            .listStyle(.sidebar)
            .navigationSplitViewColumnWidth(min: 200, ideal: 220)
            .safeAreaInset(edge: .bottom) {
                StatusFooter().padding(12)
            }
        } detail: {
            switch selection {
            case .launcher: LauncherView()
            case .monitor:  MonitorView()
            case .view3d:   RobotView3D()
            case .logs:     LogsView()
            }
        }
    }
}

struct StatusFooter: View {
    @EnvironmentObject var state: AppState
    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 8) {
                Circle()
                    .fill(color)
                    .frame(width: 9, height: 9)
                Text(state.containerStatus.label)
                    .font(.caption.weight(.medium))
                Spacer()
            }
            HStack(spacing: 8) {
                Circle()
                    .fill(state.bridgeConnected ? .green : .secondary.opacity(0.4))
                    .frame(width: 9, height: 9)
                Text(state.bridgeConnected ? "Bridge live" : "Bridge offline")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
        }
    }
    var color: Color {
        switch state.containerStatus {
        case .running: return .green
        case .stopped: return .red
        case .working: return .yellow
        case .unknown: return .gray
        }
    }
}
