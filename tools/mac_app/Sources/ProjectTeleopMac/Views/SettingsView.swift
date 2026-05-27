import SwiftUI

struct SettingsView: View {
    @EnvironmentObject var state: AppState
    @State private var draft: String = ""

    var body: some View {
        Form {
            Section("Repository") {
                TextField("project_teleop path", text: $draft, axis: .vertical)
                    .textFieldStyle(.roundedBorder)
                    .lineLimit(2)
                HStack {
                    Button("Choose…") { pick() }
                    Spacer()
                    Button("Apply") { state.repoPath = draft }
                        .keyboardShortcut(.defaultAction)
                        .disabled(draft.isEmpty || draft == state.repoPath)
                }
            }
            Section("Resolved paths") {
                LabeledContent("launcher", value: state.launcherURL.path)
                    .font(.system(.callout, design: .monospaced))
                LabeledContent("sessions", value: state.sessionsDir.path)
                    .font(.system(.callout, design: .monospaced))
            }
        }
        .padding()
        .onAppear { draft = state.repoPath }
    }

    private func pick() {
        let panel = NSOpenPanel()
        panel.canChooseDirectories = true
        panel.canChooseFiles = false
        panel.allowsMultipleSelection = false
        if panel.runModal() == .OK, let url = panel.url {
            draft = url.path
        }
    }
}
