import SwiftUI
import Charts

struct LogsView: View {
    @EnvironmentObject var state: AppState
    @State private var sessions: [SessionFile] = []
    @State private var selected: SessionFile?
    @State private var log: SessionLog?
    @State private var loading = false
    @State private var seriesPick: SeriesPick = .robotJointsDeg

    enum SeriesPick: String, CaseIterable, Identifiable {
        case robotJointsDeg = "Robot joints (°)"
        case rosCmdJointsDeg = "ROS cmd joints (°)"
        case latencyMs = "Latency (ms)"
        var id: String { rawValue }
        var keys: [String] {
            switch self {
            case .robotJointsDeg:
                return ["robot_j1_deg", "robot_j2_deg", "robot_j3_deg", "robot_j4_deg"]
            case .rosCmdJointsDeg:
                return ["ros_cmd_j1_deg", "ros_cmd_j2_deg", "ros_cmd_j3_deg", "ros_cmd_j4_deg"]
            case .latencyMs:
                return ["true_end_to_end_ms", "command_latency_ms", "motion_execution_ms"]
            }
        }
    }

    var body: some View {
        HStack(spacing: 0) {
            sidebar
                .frame(width: 280)
            Divider()
            detail
                .frame(maxWidth: .infinity, maxHeight: .infinity)
        }
        .onAppear(perform: refresh)
    }

    private var sidebar: some View {
        VStack(alignment: .leading, spacing: 0) {
            HStack {
                Text("Sessions").font(.headline)
                Spacer()
                Button(action: refresh) {
                    Image(systemName: "arrow.clockwise")
                }
                .buttonStyle(.borderless)
            }
            .padding(.horizontal, 14).padding(.top, 14).padding(.bottom, 8)

            List(sessions, selection: $selected) { f in
                VStack(alignment: .leading, spacing: 2) {
                    Text(f.name)
                        .font(.system(.callout, design: .monospaced))
                        .lineLimit(1)
                    Text(f.modified, format: .dateTime.month().day().hour().minute())
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }
                .tag(f)
            }
            .listStyle(.sidebar)
        }
        .onChange(of: selected) { f in
            guard let f else { log = nil; return }
            load(f)
        }
    }

    private var detail: some View {
        VStack(alignment: .leading, spacing: 14) {
            HStack(alignment: .firstTextBaseline) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("Session Log").font(.largeTitle.weight(.semibold))
                    if let log {
                        Text(log.file.lastPathComponent)
                            .font(.callout.monospaced())
                            .foregroundStyle(.secondary)
                    } else {
                        Text("Pick a session from the left sidebar.")
                            .foregroundStyle(.secondary)
                    }
                }
                Spacer()
                Picker("", selection: $seriesPick) {
                    ForEach(SeriesPick.allCases) { Text($0.rawValue).tag($0) }
                }
                .pickerStyle(.segmented)
                .frame(width: 460)
                .disabled(log == nil)
            }
            .padding(.horizontal, 24).padding(.top, 24)

            if loading {
                ProgressView().frame(maxWidth: .infinity, maxHeight: .infinity)
            } else if let log {
                chart(for: log)
                    .padding(.horizontal, 24)
                statsRow(for: log)
                    .padding(.horizontal, 24)
                    .padding(.bottom, 24)
            } else {
                Spacer()
            }
        }
    }

    private func chart(for log: SessionLog) -> some View {
        let keys = seriesPick.keys
        let colors: [Color] = [.blue, .green, .orange, .pink, .purple, .red]
        return Chart {
            ForEach(Array(keys.enumerated()), id: \.offset) { idx, key in
                ForEach(Array(log.rows.enumerated()), id: \.offset) { _, row in
                    if let v = row.values[key] {
                        LineMark(
                            x: .value("t", row.elapsed),
                            y: .value(key, v)
                        )
                        .foregroundStyle(by: .value("series", key))
                    }
                }
            }
        }
        .chartForegroundStyleScale(domain: keys, range: Array(colors.prefix(keys.count)))
        .chartXAxisLabel("elapsed_sec")
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .padding(12)
        .background(RoundedRectangle(cornerRadius: 12).fill(Color(nsColor: .controlBackgroundColor)))
    }

    private func statsRow(for log: SessionLog) -> some View {
        HStack(spacing: 14) {
            stat(title: "Rows", value: "\(log.rows.count)")
            if let first = log.rows.first?.elapsed, let last = log.rows.last?.elapsed {
                stat(title: "Duration", value: String(format: "%.1f s", last - first))
            }
            if let avgLat = average(log: log, key: "true_end_to_end_ms") {
                stat(title: "Avg E2E", value: String(format: "%.1f ms", avgLat))
            }
            if let maxLat = maximum(log: log, key: "true_end_to_end_ms") {
                stat(title: "Max E2E", value: String(format: "%.1f ms", maxLat))
            }
            Spacer()
        }
    }

    private func stat(title: String, value: String) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(title).font(.caption).foregroundStyle(.secondary).textCase(.uppercase)
            Text(value).font(.title3.weight(.semibold)).monospacedDigit()
        }
        .padding(10)
        .background(RoundedRectangle(cornerRadius: 8).fill(Color(nsColor: .controlBackgroundColor)))
    }

    private func average(log: SessionLog, key: String) -> Double? {
        let xs = log.rows.compactMap { $0.values[key] }
        guard !xs.isEmpty else { return nil }
        return xs.reduce(0, +) / Double(xs.count)
    }
    private func maximum(log: SessionLog, key: String) -> Double? {
        log.rows.compactMap { $0.values[key] }.max()
    }

    // MARK: - actions

    private func refresh() {
        sessions = SessionLogReader.listSessions(in: state.sessionsDir)
        if selected == nil { selected = sessions.first }
    }

    private func load(_ file: SessionFile) {
        loading = true
        log = nil
        Task.detached {
            let result = try? SessionLogReader.load(file.url)
            await MainActor.run {
                self.log = result
                self.loading = false
            }
        }
    }
}
