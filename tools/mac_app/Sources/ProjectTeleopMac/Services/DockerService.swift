import Foundation

struct LauncherProcessError: LocalizedError {
    let status: Int32
    let output: String

    var errorDescription: String? {
        let trimmed = output.trimmingCharacters(in: .whitespacesAndNewlines)
        if trimmed.isEmpty {
            return "launcher exited with code \(status)"
        }
        return trimmed
    }
}

/// Spawns `launcher.sh` and streams stdout/stderr line-by-line.
final class DockerService {
    private var bridgeProcess: Process?

    /// One-shot run: collect full stdout, return as a single string.
    func runOnce(launcher: URL, arguments: [String]) async throws -> String {
        try await withCheckedThrowingContinuation { (cont: CheckedContinuation<String, Error>) in
            DispatchQueue.global(qos: .userInitiated).async {
                let process = Process()
                let pipe = Pipe()
                process.executableURL = URL(fileURLWithPath: "/bin/bash")
                process.arguments = [launcher.path] + arguments
                process.standardOutput = pipe
                process.standardError = pipe
                process.environment = Self.shellEnvironment()
                do {
                    try process.run()
                } catch {
                    cont.resume(throwing: error)
                    return
                }
                process.waitUntilExit()
                let data = pipe.fileHandleForReading.readDataToEndOfFile()
                let output = String(data: data, encoding: .utf8) ?? ""
                if process.terminationStatus == 0 {
                    cont.resume(returning: output)
                } else {
                    cont.resume(throwing: LauncherProcessError(status: process.terminationStatus, output: output))
                }
            }
        }
    }

    /// Streams stdout/stderr as they arrive. Caller iterates until completion.
    func spawn(launcher: URL, arguments: [String]) throws -> AsyncStream<String> {
        let process = Process()
        let pipe = Pipe()
        process.executableURL = URL(fileURLWithPath: "/bin/bash")
        process.arguments = [launcher.path] + arguments
        process.standardOutput = pipe
        process.standardError = pipe
        process.environment = Self.shellEnvironment()

        if arguments.first == "bridge" {
            bridgeProcess = process
        }

        let handle = pipe.fileHandleForReading
        try process.run()

        return AsyncStream<String> { continuation in
            let buffer = LineBuffer()
            let finishLock = NSLock()
            var didFinish = false

            func finishStream() {
                finishLock.lock()
                if didFinish {
                    finishLock.unlock()
                    return
                }
                didFinish = true
                finishLock.unlock()

                handle.readabilityHandler = nil
                let remaining = handle.readDataToEndOfFile()
                if !remaining.isEmpty {
                    for line in buffer.append(remaining) {
                        continuation.yield(line)
                    }
                }
                let trailing = buffer.flush()
                if !trailing.isEmpty { continuation.yield(trailing) }
                continuation.finish()
            }

            handle.readabilityHandler = { fh in
                let data = fh.availableData
                if data.isEmpty {
                    finishStream()
                } else {
                    for line in buffer.append(data) {
                        continuation.yield(line)
                    }
                }
            }

            continuation.onTermination = { _ in
                handle.readabilityHandler = nil
                if process.isRunning { process.terminate() }
            }

            process.terminationHandler = { _ in
                if process.terminationStatus != 0 {
                    continuation.yield("launcher exited with code \(process.terminationStatus)")
                }
                finishStream()
            }
        }
    }

    func terminateBridge() {
        if let process = bridgeProcess, process.isRunning {
            process.terminate()
        }
        bridgeProcess = nil
    }

    private static func shellEnvironment() -> [String: String] {
        var env = ProcessInfo.processInfo.environment
        // Docker Desktop's CLI lives in /usr/local/bin, sometimes /opt/homebrew/bin.
        let path = env["PATH"] ?? ""
        let extras = ["/usr/local/bin", "/opt/homebrew/bin", "/Applications/Docker.app/Contents/Resources/bin"]
        let merged = (extras + path.split(separator: ":").map(String.init))
            .reduce(into: [String]()) { acc, item in
                if !acc.contains(item) { acc.append(item) }
            }
            .joined(separator: ":")
        env["PATH"] = merged
        return env
    }
}

/// Splits arbitrary chunked data into UTF-8 lines.
final class LineBuffer {
    private var pending = Data()

    func append(_ data: Data) -> [String] {
        pending.append(data)
        var lines: [String] = []
        while let nl = pending.firstIndex(of: 0x0a) {
            let lineData = pending.subdata(in: pending.startIndex..<nl)
            pending.removeSubrange(pending.startIndex...nl)
            if let s = String(data: lineData, encoding: .utf8) {
                lines.append(s)
            }
        }
        return lines
    }

    func flush() -> String {
        defer { pending.removeAll() }
        return String(data: pending, encoding: .utf8) ?? ""
    }
}
