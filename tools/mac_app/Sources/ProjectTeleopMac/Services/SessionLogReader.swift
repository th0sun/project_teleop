import Foundation

struct SessionFile: Identifiable, Hashable {
    let url: URL
    var id: URL { url }
    var name: String { url.lastPathComponent }
    var modified: Date {
        (try? url.resourceValues(forKeys: [.contentModificationDateKey]).contentModificationDate) ?? .distantPast
    }
}

struct SessionRow {
    var elapsed: Double
    var values: [String: Double]
}

struct SessionLog {
    var file: URL
    var headers: [String]
    var rows: [SessionRow]
}

enum SessionLogReader {
    static func listSessions(in dir: URL) -> [SessionFile] {
        let fm = FileManager.default
        guard let items = try? fm.contentsOfDirectory(
            at: dir,
            includingPropertiesForKeys: [.contentModificationDateKey],
            options: [.skipsHiddenFiles]
        ) else { return [] }
        return items
            .filter { $0.pathExtension.lowercased() == "csv" }
            .map(SessionFile.init)
            .sorted { $0.modified > $1.modified }
    }

    static func load(_ url: URL) throws -> SessionLog {
        let raw = try String(contentsOf: url, encoding: .utf8)
        let allLines = raw.split(separator: "\n", omittingEmptySubsequences: false)
        guard let headerLine = allLines.first else {
            return SessionLog(file: url, headers: [], rows: [])
        }
        let headers = parseCSVLine(String(headerLine))
        let elapsedIdx = headers.firstIndex(of: "elapsed_sec")

        let interestingColumns: Set<String> = [
            "robot_j1_deg", "robot_j2_deg", "robot_j3_deg", "robot_j4_deg",
            "ros_cmd_j1_deg", "ros_cmd_j2_deg", "ros_cmd_j3_deg", "ros_cmd_j4_deg",
            "true_end_to_end_ms", "command_latency_ms", "motion_execution_ms",
            "velocity_mag_rad_s", "robot_mode", "error_status",
        ]
        let columnIdx: [(String, Int)] = headers.enumerated().compactMap { idx, name in
            interestingColumns.contains(name) ? (name, idx) : nil
        }

        var rows: [SessionRow] = []
        rows.reserveCapacity(max(0, allLines.count - 1))

        for raw in allLines.dropFirst() {
            if raw.isEmpty { continue }
            let cols = parseCSVLine(String(raw))
            guard let eIdx = elapsedIdx, eIdx < cols.count,
                  let elapsed = Double(cols[eIdx]) else { continue }
            var values: [String: Double] = [:]
            for (name, idx) in columnIdx where idx < cols.count {
                if let v = Double(cols[idx]) { values[name] = v }
            }
            rows.append(SessionRow(elapsed: elapsed, values: values))
        }
        return SessionLog(file: url, headers: headers, rows: rows)
    }

    private static func parseCSVLine(_ s: String) -> [String] {
        var fields: [String] = []
        var current = ""
        var inQuotes = false
        var iter = s.makeIterator()
        while let ch = iter.next() {
            if ch == "\"" {
                inQuotes.toggle()
            } else if ch == "," && !inQuotes {
                fields.append(current)
                current = ""
            } else if ch == "\r" {
                continue
            } else {
                current.append(ch)
            }
        }
        fields.append(current)
        return fields
    }
}
