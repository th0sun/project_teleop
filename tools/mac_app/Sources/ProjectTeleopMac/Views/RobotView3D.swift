import SwiftUI
import SceneKit
import simd

struct RobotView3D: View {
    @EnvironmentObject var state: AppState

    var body: some View {
        VStack(spacing: 0) {
            header
                .padding(20)
            SceneViewWrapper(controller: state.scene)
                .background(Color(nsColor: .black))
        }
        .onAppear {
            state.scene.configure(repoPath: state.repoPath)
            state.scene.updateActual(state.jointPositionsDeg)
            state.scene.updateUnity(state.unityJointPositionsDeg)
        }
        .onChange(of: state.repoPath) { repoPath in
            state.scene.configure(repoPath: repoPath)
            state.scene.updateActual(state.jointPositionsDeg)
            state.scene.updateUnity(state.unityJointPositionsDeg)
        }
        .onChange(of: state.jointPositionsDeg) { joints in
            state.scene.updateActual(joints)
        }
        .onChange(of: state.unityJointPositionsDeg ?? []) { target in
            state.scene.updateUnity(target.isEmpty ? nil : target)
        }
        .onChange(of: state.lastUnityJointUpdate) { _ in
            state.scene.markUnityFresh()
        }
    }

    private var header: some View {
        HStack(alignment: .firstTextBaseline) {
            VStack(alignment: .leading, spacing: 4) {
                Text("MG400 - Live 3D").font(.largeTitle.weight(.semibold))
                Text("MG400 mesh pipeline from Mac Simulator. Real color = /joint_states, transparent = /unity/teleop_sample target.")
                    .foregroundStyle(.secondary)
                Text(state.scene.meshStatus)
                    .font(.caption.monospaced())
                    .foregroundStyle(state.scene.meshStatus.hasPrefix("loaded") ? Color.secondary : Color.orange)
            }
            Spacer()
            BridgeBadge()
        }
    }
}

struct SceneViewWrapper: NSViewRepresentable {
    let controller: MG400SceneController
    func makeNSView(context: Context) -> SCNView {
        let view = SCNView()
        view.scene = controller.scene
        view.backgroundColor = .clear
        view.allowsCameraControl = true
        view.autoenablesDefaultLighting = false
        view.antialiasingMode = .multisampling2X
        view.preferredFramesPerSecond = 120
        return view
    }
    func updateNSView(_ nsView: SCNView, context: Context) {}
}

@MainActor
final class MG400SceneController: ObservableObject {
    let scene = SCNScene()
    @Published var meshStatus = "loading MG400 meshes"

    private let robotRoot = SCNNode()
    private let actualRoot = SCNNode()
    private let unityRoot = SCNNode()
    private var actualLinks: [String: SCNNode] = [:]
    private var unityLinks: [String: SCNNode] = [:]
    private var configuredRepoPath = ""
    private var lastActualJoints: [Double] = []
    private var lastUnityJoints: [Double] = []
    private var unityStaleTask: Task<Void, Never>?
    // Mock dobot motion for a single joint_cmd can take several seconds
    // (we have seen 7+ s in mock-sim mode). With a 1-second timeout the cyan
    // target ghost vanishes long before the real robot has finished tracking
    // toward it, which looks like "robot moves briefly then stops". Hold the
    // ghost long enough to cover a normal mock motion.
    private let unityStaleSeconds: UInt64 = 10

    private let drawOrder = [
        "base_link", "link1", "link2_1", "link2_2", "link3_1", "link3_2",
        "link4_1", "link4_2", "link5", "link4_3", "flange",
    ]

    private let meshFiles: [String: String] = [
        "base_link": "base_link.dae",
        "link1": "link1.dae",
        "link2_1": "link2_1.dae",
        "link2_2": "link2_2.dae",
        "link3_1": "link3_1.dae",
        "link3_2": "link3_2.dae",
        "link4_1": "link4_1.dae",
        "link4_2": "link4_2.dae",
        "link4_3": "link4_3.dae",
        "link5": "link5.dae",
        "flange": "flange.dae",
    ]

    private let linkColors: [String: NSColor] = [
        "base_link": NSColor(calibratedRed: 0.15, green: 0.15, blue: 0.15, alpha: 1.0),
        "link1": NSColor(calibratedRed: 0.90, green: 0.90, blue: 0.90, alpha: 1.0),
        "link2_1": NSColor(calibratedRed: 0.90, green: 0.90, blue: 0.90, alpha: 1.0),
        "link2_2": NSColor(calibratedRed: 0.60, green: 0.60, blue: 0.60, alpha: 1.0),
        "link3_1": NSColor(calibratedRed: 0.90, green: 0.90, blue: 0.90, alpha: 1.0),
        "link3_2": NSColor(calibratedRed: 0.60, green: 0.60, blue: 0.60, alpha: 1.0),
        "link4_1": NSColor(calibratedRed: 0.90, green: 0.90, blue: 0.90, alpha: 1.0),
        "link4_2": NSColor(calibratedRed: 0.60, green: 0.60, blue: 0.60, alpha: 1.0),
        "link4_3": NSColor(calibratedRed: 0.20, green: 0.20, blue: 0.20, alpha: 1.0),
        "link5": NSColor(calibratedRed: 0.25, green: 0.25, blue: 0.25, alpha: 1.0),
        "flange": NSColor(calibratedRed: 0.40, green: 0.40, blue: 0.40, alpha: 1.0),
    ]

    init() {
        scene.background.contents = NSColor.black

        robotRoot.eulerAngles = SCNVector3(-Float.pi / 2.0, 0, 0)
        scene.rootNode.addChildNode(robotRoot)
        robotRoot.addChildNode(actualRoot)
        robotRoot.addChildNode(unityRoot)

        unityRoot.opacity = 1.0
        unityRoot.isHidden = true

        addGrid()
        addCamera()
        addLights()
    }

    func configure(repoPath: String) {
        guard repoPath != configuredRepoPath else { return }
        configuredRepoPath = repoPath

        actualRoot.childNodes.forEach { $0.removeFromParentNode() }
        unityRoot.childNodes.forEach { $0.removeFromParentNode() }
        actualLinks.removeAll()
        unityLinks.removeAll()

        let meshesDir = URL(fileURLWithPath: repoPath)
            .appendingPathComponent("src/dobot_mg400/mg400_description/meshes")
        var loaded = 0

        for link in drawOrder {
            guard let fileName = meshFiles[link] else { continue }
            let url = meshesDir.appendingPathComponent(fileName)
            guard let node = MG400DAELoader.loadMeshNode(
                url: url,
                color: linkColors[link] ?? NSColor(calibratedWhite: 0.7, alpha: 1.0)
            ) else { continue }

            let actualNode = node
            actualNode.name = link
            actualRoot.addChildNode(actualNode)
            actualLinks[link] = actualNode

            let targetNode = cloneWithCopiedGeometry(node)
            targetNode.name = "\(link)_unity_target"
            makeTransparent(node: targetNode, color: NSColor.systemCyan)
            unityRoot.addChildNode(targetNode)
            unityLinks[link] = targetNode
            loaded += 1
        }

        meshStatus = loaded == drawOrder.count
            ? "loaded \(loaded) MG400 mesh links"
            : "loaded \(loaded)/\(drawOrder.count) mesh links from \(meshesDir.path)"
    }

    func updateActual(_ joints: [Double]) {
        guard !jointsApproxEqual(joints, lastActualJoints, tol: 0.02) else { return }
        lastActualJoints = joints
        apply(joints: joints, to: actualLinks)
    }

    func updateUnity(_ joints: [Double]?) {
        guard let joints, !joints.isEmpty else {
            unityRoot.isHidden = true
            lastUnityJoints = []
            return
        }
        if jointsApproxEqual(joints, lastUnityJoints, tol: 0.02), !unityRoot.isHidden {
            return
        }
        lastUnityJoints = joints
        unityRoot.isHidden = false
        apply(joints: joints, to: unityLinks)
    }

    func markUnityFresh() {
        unityStaleTask?.cancel()
        let stale = unityStaleSeconds
        unityStaleTask = Task { @MainActor [weak self] in
            try? await Task.sleep(nanoseconds: stale * 1_000_000_000)
            guard let self, !Task.isCancelled else { return }
            self.unityRoot.isHidden = true
            self.lastUnityJoints = []
        }
    }

    private func jointsApproxEqual(_ a: [Double], _ b: [Double], tol: Double) -> Bool {
        guard a.count == b.count else { return false }
        for i in 0..<a.count where abs(a[i] - b[i]) > tol { return false }
        return true
    }

    private func apply(joints: [Double], to links: [String: SCNNode]) {
        guard !links.isEmpty else { return }
        let j1 = degToRad(Float(joints.indices.contains(0) ? joints[0] : 0))
        let j2 = degToRad(Float(joints.indices.contains(1) ? joints[1] : 0))
        let j3 = degToRad(Float(joints.indices.contains(2) ? joints[2] : 0))
        let j4 = degToRad(Float(joints.indices.contains(3) ? joints[3] : 0))

        let transforms = computeTransforms(j1: j1, j2: j2, j3: j3, j4: j4)
        SCNTransaction.begin()
        SCNTransaction.animationDuration = 0
        SCNTransaction.disableActions = true
        for (link, transform) in transforms {
            links[link]?.simdTransform = transform
        }
        SCNTransaction.commit()
    }

    private func computeTransforms(j1: Float, j2: Float, j3: Float, j4: Float) -> [String: simd_float4x4] {
        var t: [String: simd_float4x4] = ["base_link": matrix_identity_float4x4]
        t["link1"] = t["base_link"]! * translate(-0.005, 0, 0.109) * rotateZ(j1)
        t["link2_1"] = t["link1"]! * translate(0.0435, -0.035775, 0.119) * rotateY(j2)
        t["link2_2"] = t["link1"]! * translate(0.0045288568, -0.0305, 0.1415) * rotateY(j2)
        t["link3_1"] = t["link2_1"]! * translate(-0.001051257, 0.035774876, 0.175001164) * rotateY(j3 - j2)
        t["link3_2"] = t["link2_2"]! * translate(-0.00105, 0.0065, 0.175) * rotateY(-j2)
        t["link4_1"] = t["link3_1"]! * translate(0.175, -0.017, 0.00325) * rotateY(-j3)
        t["link4_2"] = t["link3_2"]! * translate(0.0679, 0.0005, 0.011972) * rotateY(j3)
        t["link5"] = t["link4_1"]! * translate(0.066, 0.017, 0.031)
        t["link4_3"] = t["link5"]! * translate(0, 0, 0.002)
        t["flange"] = t["link5"]! * translate(0, 0, -0.084) * rotateZ(j4)
        return t
    }

    private func makeTransparent(node: SCNNode, color: NSColor) {
        node.renderingOrder = 10
        if let geometry = node.geometry {
            geometry.materials = geometry.materials.map { material in
                let copy = material.copy() as! SCNMaterial
                copy.diffuse.contents = color.withAlphaComponent(0.28)
                copy.emission.contents = color.withAlphaComponent(0.06)
                copy.specular.contents = NSColor.clear
                copy.lightingModel = .constant
                copy.blendMode = .alpha
                copy.transparency = 0.28
                copy.fillMode = .fill
                copy.isDoubleSided = true
                copy.readsFromDepthBuffer = true
                copy.writesToDepthBuffer = false
                return copy
            }
        }
        node.childNodes.forEach { makeTransparent(node: $0, color: color) }
    }

    private func cloneWithCopiedGeometry(_ node: SCNNode) -> SCNNode {
        let clone = node.clone()
        copyGeometryRecursively(clone)
        return clone
    }

    private func copyGeometryRecursively(_ node: SCNNode) {
        if let geometry = node.geometry {
            node.geometry = geometry.copy() as? SCNGeometry
        }
        node.childNodes.forEach { copyGeometryRecursively($0) }
    }

    private func addGrid() {
        let grid = SCNNode()
        for i in -6...6 {
            let xLine = SCNNode(geometry: SCNBox(width: 1.2, height: 0.0015, length: 0.0015, chamferRadius: 0))
            xLine.geometry?.firstMaterial?.diffuse.contents = NSColor(calibratedWhite: 0.22, alpha: 1)
            xLine.position = SCNVector3(0, Float(i) * 0.1, 0)
            grid.addChildNode(xLine)

            let yLine = SCNNode(geometry: SCNBox(width: 0.0015, height: 1.2, length: 0.0015, chamferRadius: 0))
            yLine.geometry?.firstMaterial?.diffuse.contents = NSColor(calibratedWhite: 0.22, alpha: 1)
            yLine.position = SCNVector3(Float(i) * 0.1, 0, 0)
            grid.addChildNode(yLine)
        }
        robotRoot.addChildNode(grid)
    }

    private func addCamera() {
        let cameraNode = SCNNode()
        cameraNode.camera = SCNCamera()
        cameraNode.camera?.zNear = 0.01
        cameraNode.camera?.zFar = 10
        cameraNode.position = SCNVector3(0.85, 0.85, 0.65)
        cameraNode.look(at: SCNVector3(0.18, 0.0, 0.24))
        scene.rootNode.addChildNode(cameraNode)
    }

    private func addLights() {
        let ambient = SCNNode()
        ambient.light = SCNLight()
        ambient.light?.type = .ambient
        ambient.light?.color = NSColor(calibratedWhite: 0.32, alpha: 1)
        scene.rootNode.addChildNode(ambient)

        let key = SCNNode()
        key.light = SCNLight()
        key.light?.type = .directional
        key.light?.color = NSColor.white
        key.eulerAngles = SCNVector3(-Float.pi / 3, Float.pi / 4, 0)
        scene.rootNode.addChildNode(key)
    }
}

private enum MG400DAELoader {
    static func loadMeshNode(url: URL, color: NSColor) -> SCNNode? {
        guard FileManager.default.fileExists(atPath: url.path),
              let parser = XMLParser(contentsOf: url) else {
            return nil
        }
        let loader = DAEGeometryParser()
        parser.delegate = loader
        parser.shouldProcessNamespaces = false
        guard parser.parse() else { return nil }
        return loader.makeNode(color: color)
    }
}

private final class DAEGeometryParser: NSObject, XMLParserDelegate {
    private struct DAEInput {
        let semantic: String
        let source: String
        let offset: Int
    }

    private struct DAETriangles {
        var inputs: [DAEInput] = []
        var indices: [Int] = []
    }

    private struct DAEGeometry {
        var triangles: [DAETriangles] = []
    }

    private enum TextMode {
        case none
        case floatArray
        case triangleIndices
        case matrix
    }

    private var sources: [String: [Float]] = [:]
    private var vertexPositionSources: [String: String] = [:]
    private var geometries: [String: DAEGeometry] = [:]
    private var instanceTransforms: [String: [simd_float4x4]] = [:]

    private var currentSourceID: String?
    private var currentVerticesID: String?
    private var currentGeometryID: String?
    private var currentTriangles: DAETriangles?
    private var nodeTransforms: [simd_float4x4] = []

    private var textMode: TextMode = .none
    private var textBuffer = ""

    func parser(
        _ parser: XMLParser,
        didStartElement elementName: String,
        namespaceURI: String?,
        qualifiedName qName: String?,
        attributes attributeDict: [String: String] = [:]
    ) {
        switch elementName {
        case "geometry":
            if let id = attributeDict["id"] {
                currentGeometryID = id
                geometries[id] = DAEGeometry()
            }
        case "source":
            currentSourceID = attributeDict["id"]
        case "float_array":
            if currentSourceID != nil {
                textMode = .floatArray
                textBuffer = ""
            }
        case "vertices":
            currentVerticesID = attributeDict["id"]
        case "input":
            handleInput(attributeDict)
        case "triangles":
            currentTriangles = DAETriangles()
        case "p":
            if currentTriangles != nil {
                textMode = .triangleIndices
                textBuffer = ""
            }
        case "node":
            nodeTransforms.append(nodeTransforms.last ?? matrix_identity_float4x4)
        case "matrix":
            if !nodeTransforms.isEmpty {
                textMode = .matrix
                textBuffer = ""
            }
        case "instance_geometry":
            if let source = attributeDict["url"] {
                let geometryID = stripHash(source)
                instanceTransforms[geometryID, default: []].append(nodeTransforms.last ?? matrix_identity_float4x4)
            }
        default:
            break
        }
    }

    func parser(_ parser: XMLParser, foundCharacters string: String) {
        if textMode != .none {
            textBuffer += string
        }
    }

    func parser(
        _ parser: XMLParser,
        didEndElement elementName: String,
        namespaceURI: String?,
        qualifiedName qName: String?
    ) {
        switch elementName {
        case "float_array":
            if let id = currentSourceID {
                sources[id] = parseFloats(textBuffer)
            }
            textMode = .none
            textBuffer = ""
        case "source":
            currentSourceID = nil
        case "vertices":
            currentVerticesID = nil
        case "p":
            currentTriangles?.indices = parseInts(textBuffer)
            textMode = .none
            textBuffer = ""
        case "triangles":
            if let geometryID = currentGeometryID, let triangles = currentTriangles {
                geometries[geometryID, default: DAEGeometry()].triangles.append(triangles)
            }
            currentTriangles = nil
        case "geometry":
            currentGeometryID = nil
        case "matrix":
            let values = parseFloats(textBuffer)
            if values.count == 16, !nodeTransforms.isEmpty {
                let parent = nodeTransforms.count >= 2
                    ? nodeTransforms[nodeTransforms.count - 2]
                    : matrix_identity_float4x4
                nodeTransforms[nodeTransforms.count - 1] = parent * matrixFromRows(values)
            }
            textMode = .none
            textBuffer = ""
        case "node":
            if !nodeTransforms.isEmpty {
                nodeTransforms.removeLast()
            }
        default:
            break
        }
    }

    func makeNode(color: NSColor) -> SCNNode? {
        var vertices: [SCNVector3] = []
        var normals: [SCNVector3] = []
        var indices: [UInt32] = []

        for (geometryID, geometry) in geometries {
            let transforms = instanceTransforms[geometryID] ?? [matrix_identity_float4x4]
            for transform in transforms {
                append(geometry: geometry, transform: transform, vertices: &vertices, normals: &normals, indices: &indices)
            }
        }

        guard !vertices.isEmpty else { return nil }

        let material = SCNMaterial()
        material.diffuse.contents = color
        material.specular.contents = NSColor(calibratedWhite: 0.35, alpha: 1.0)
        material.lightingModel = .phong
        material.isDoubleSided = true

        let geometry = SCNGeometry(
            sources: [
                SCNGeometrySource(vertices: vertices),
                SCNGeometrySource(normals: normals),
            ],
            elements: [
                SCNGeometryElement(indices: indices, primitiveType: .triangles),
            ]
        )
        geometry.materials = [material]

        let node = SCNNode(geometry: geometry)
        node.name = "dae_mesh"
        return node
    }

    private func append(
        geometry: DAEGeometry,
        transform: simd_float4x4,
        vertices: inout [SCNVector3],
        normals: inout [SCNVector3],
        indices: inout [UInt32]
    ) {
        for triangles in geometry.triangles {
            guard let vertexInput = triangles.inputs.first(where: { $0.semantic == "VERTEX" }) else {
                continue
            }
            let stride = max((triangles.inputs.map(\.offset).max() ?? 0) + 1, 1)
            let positionSourceID = vertexPositionSources[vertexInput.source] ?? vertexInput.source
            guard let positions = sources[positionSourceID] else { continue }

            let raw = triangles.indices
            var cursor = 0
            while cursor + stride * 3 <= raw.count {
                let i0 = raw[cursor + vertexInput.offset]
                let i1 = raw[cursor + stride + vertexInput.offset]
                let i2 = raw[cursor + stride * 2 + vertexInput.offset]
                cursor += stride * 3

                guard let p0 = point(positions: positions, index: i0, transform: transform),
                      let p1 = point(positions: positions, index: i1, transform: transform),
                      let p2 = point(positions: positions, index: i2, transform: transform) else {
                    continue
                }

                let normal = faceNormal(p0, p1, p2)
                for p in [p0, p1, p2] {
                    vertices.append(p)
                    normals.append(normal)
                    indices.append(UInt32(indices.count))
                }
            }
        }
    }

    private func handleInput(_ attributes: [String: String]) {
        guard let semantic = attributes["semantic"],
              let rawSource = attributes["source"] else {
            return
        }
        let source = stripHash(rawSource)
        if let verticesID = currentVerticesID, semantic == "POSITION" {
            vertexPositionSources[verticesID] = source
            return
        }
        if currentTriangles != nil {
            let offset = Int(attributes["offset"] ?? "0") ?? 0
            currentTriangles?.inputs.append(DAEInput(semantic: semantic, source: source, offset: offset))
        }
    }

    private func point(positions: [Float], index: Int, transform: simd_float4x4) -> SCNVector3? {
        let base = index * 3
        guard base + 2 < positions.count else { return nil }
        let v = transform * SIMD4<Float>(positions[base], positions[base + 1], positions[base + 2], 1.0)
        let w = abs(v.w) > 0.000001 ? v.w : 1.0
        return SCNVector3(v.x / w, v.y / w, v.z / w)
    }

    private func faceNormal(_ a: SCNVector3, _ b: SCNVector3, _ c: SCNVector3) -> SCNVector3 {
        let va = SIMD3<Float>(Float(a.x), Float(a.y), Float(a.z))
        let vb = SIMD3<Float>(Float(b.x), Float(b.y), Float(b.z))
        let vc = SIMD3<Float>(Float(c.x), Float(c.y), Float(c.z))
        let n = simd_cross(vb - va, vc - va)
        let length = simd_length(n)
        guard length > 0.000001, length.isFinite else {
            return SCNVector3(0, 0, 1)
        }
        let unit = n / length
        return SCNVector3(unit.x, unit.y, unit.z)
    }

    private func parseFloats(_ text: String) -> [Float] {
        text.split(whereSeparator: { $0 == " " || $0 == "\n" || $0 == "\t" || $0 == "\r" })
            .compactMap { Float(String($0)) }
    }

    private func parseInts(_ text: String) -> [Int] {
        text.split(whereSeparator: { $0 == " " || $0 == "\n" || $0 == "\t" || $0 == "\r" })
            .compactMap { Int($0) }
    }

    private func stripHash(_ value: String) -> String {
        value.hasPrefix("#") ? String(value.dropFirst()) : value
    }

    private func matrixFromRows(_ values: [Float]) -> simd_float4x4 {
        simd_float4x4(
            SIMD4<Float>(values[0], values[4], values[8], values[12]),
            SIMD4<Float>(values[1], values[5], values[9], values[13]),
            SIMD4<Float>(values[2], values[6], values[10], values[14]),
            SIMD4<Float>(values[3], values[7], values[11], values[15])
        )
    }
}

private func degToRad(_ value: Float) -> Float {
    value * Float.pi / 180.0
}

private func translate(_ x: Float, _ y: Float, _ z: Float) -> simd_float4x4 {
    var m = matrix_identity_float4x4
    m.columns.3 = SIMD4<Float>(x, y, z, 1)
    return m
}

private func rotateY(_ a: Float) -> simd_float4x4 {
    let c = cos(a)
    let s = sin(a)
    return simd_float4x4(
        SIMD4<Float>( c, 0, -s, 0),
        SIMD4<Float>( 0, 1,  0, 0),
        SIMD4<Float>( s, 0,  c, 0),
        SIMD4<Float>( 0, 0,  0, 1)
    )
}

private func rotateZ(_ a: Float) -> simd_float4x4 {
    let c = cos(a)
    let s = sin(a)
    return simd_float4x4(
        SIMD4<Float>( c, s, 0, 0),
        SIMD4<Float>(-s, c, 0, 0),
        SIMD4<Float>( 0, 0, 1, 0),
        SIMD4<Float>( 0, 0, 0, 1)
    )
}
