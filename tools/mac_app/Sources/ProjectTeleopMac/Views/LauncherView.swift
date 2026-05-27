import SwiftUI

struct LauncherView: View {
    @EnvironmentObject var state: AppState

    var body: some View {
        VStack(alignment: .leading, spacing: 20) {
            header

            modeGrid

            HStack(spacing: 12) {
                Button {
                    state.start()
                } label: {
                    Label(startTitle, systemImage: startIcon)
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(PrimaryButtonStyle(tint: .green))
                .disabled(state.containerStatus == .working)

                Button {
                    state.stop()
                } label: {
                    Label("Stop", systemImage: "stop.fill")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(PrimaryButtonStyle(tint: .red))
                .disabled(state.containerStatus == .stopped || state.containerStatus == .working)

                Button {
                    Task { await state.refreshStatus() }
                } label: {
                    Label("Refresh", systemImage: "arrow.clockwise")
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(PrimaryButtonStyle(tint: .blue))
            }

            consolePanel
        }
        .padding(24)
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topLeading)
    }

    private var startTitle: String {
        state.containerStatus == .running ? "Restart" : "Start"
    }

    private var startIcon: String {
        state.containerStatus == .running ? "arrow.clockwise" : "play.fill"
    }

    private var header: some View {
        HStack(alignment: .firstTextBaseline) {
            VStack(alignment: .leading, spacing: 4) {
                Text("Launcher").font(.largeTitle.weight(.semibold))
                Text("Run the same ROS2 stack as `./startup` without Terminal panes.")
                    .foregroundStyle(.secondary)
            }
            Spacer()
            StatusBadge()
        }
    }

    private var modeGrid: some View {
        let cols = [GridItem(.adaptive(minimum: 220), spacing: 12)]
        return LazyVGrid(columns: cols, spacing: 12) {
            ForEach(LaunchMode.allCases) { mode in
                ModeCard(mode: mode, selected: state.selectedMode == mode) {
                    state.selectedMode = mode
                }
            }
        }
    }

    private var consolePanel: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("Console").font(.headline)
                Picker("Console pane", selection: $state.selectedConsolePane) {
                    ForEach(ConsolePane.allCases) { pane in
                        Text(pane.label).tag(pane)
                    }
                }
                .pickerStyle(.segmented)
                .labelsHidden()
                .frame(maxWidth: 520)
                .onChange(of: state.selectedConsolePane) { _ in
                    Task { await state.refreshSelectedConsole() }
                }
                Spacer()
                Button(state.loadingServiceLog ? "Loading" : "Refresh") {
                    Task { await state.refreshSelectedConsole() }
                }
                .buttonStyle(.borderless)
                .foregroundStyle(.secondary)
                .disabled(state.loadingServiceLog)
                Button("Clear") { state.clearSelectedConsole() }
                    .buttonStyle(.borderless)
                    .foregroundStyle(.secondary)
            }
            ScrollViewReader { proxy in
                ScrollView {
                    LazyVStack(alignment: .leading, spacing: 0) {
                        ForEach(Array(state.currentConsoleLines.enumerated()), id: \.offset) { idx, line in
                            Text(line)
                                .font(.system(.callout, design: .monospaced))
                                .frame(maxWidth: .infinity, alignment: .leading)
                                .padding(.vertical, 1)
                                .id(idx)
                        }
                    }
                    .padding(12)
                }
                .frame(maxHeight: .infinity)
                .background(Color(nsColor: .underPageBackgroundColor))
                .clipShape(RoundedRectangle(cornerRadius: 12))
                .onChange(of: state.currentConsoleLines.count) { _ in
                    if let last = state.currentConsoleLines.indices.last {
                        withAnimation(.easeOut(duration: 0.15)) {
                            proxy.scrollTo(last, anchor: .bottom)
                        }
                    }
                }
            }
        }
    }
}

private struct ModeCard: View {
    let mode: LaunchMode
    let selected: Bool
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            VStack(alignment: .leading, spacing: 10) {
                HStack {
                    Image(systemName: mode.systemImage)
                        .font(.title2)
                        .foregroundStyle(selected ? .white : .accentColor)
                    Spacer()
                    if selected {
                        Image(systemName: "checkmark.circle.fill")
                            .foregroundStyle(.white)
                    }
                }
                Text(mode.label).font(.headline)
                Text(mode.rawValue)
                    .font(.system(.caption, design: .monospaced))
                    .foregroundStyle(selected ? .white.opacity(0.8) : .secondary)
            }
            .padding(14)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(
                RoundedRectangle(cornerRadius: 14)
                    .fill(selected ? Color.accentColor : Color(nsColor: .controlBackgroundColor))
            )
            .foregroundStyle(selected ? .white : .primary)
            .overlay(
                RoundedRectangle(cornerRadius: 14)
                    .strokeBorder(selected ? .clear : Color.secondary.opacity(0.2))
            )
        }
        .buttonStyle(.plain)
    }
}

struct StatusBadge: View {
    @EnvironmentObject var state: AppState
    var body: some View {
        HStack(spacing: 8) {
            Circle().fill(color).frame(width: 10, height: 10)
            Text(state.containerStatus.label).font(.subheadline.weight(.medium))
        }
        .padding(.horizontal, 12)
        .padding(.vertical, 6)
        .background(Capsule().fill(Color(nsColor: .controlBackgroundColor)))
        .overlay(Capsule().strokeBorder(Color.secondary.opacity(0.2)))
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

struct PrimaryButtonStyle: ButtonStyle {
    var tint: Color
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.body.weight(.semibold))
            .padding(.vertical, 10)
            .foregroundStyle(.white)
            .background(
                RoundedRectangle(cornerRadius: 10)
                    .fill(tint.opacity(configuration.isPressed ? 0.7 : 1.0))
            )
            .opacity(configuration.isPressed ? 0.85 : 1)
    }
}
