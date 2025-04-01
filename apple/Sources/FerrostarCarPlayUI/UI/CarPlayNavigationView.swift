import FerrostarCore
import FerrostarCoreFFI
import FerrostarMapLibreUI
import FerrostarSwiftUI
import MapLibre
import MapLibreSwiftDSL
import MapLibreSwiftUI
import SwiftUI

public struct CarPlayNavigationView: View,
    SpeedLimitViewHost, NavigationViewConfigurable
{
    @StateObject var ferrostarCore: FerrostarCore
    @Environment(\.navigationFormatterCollection) var formatterCollection: any FormatterCollection

    let styleURL: URL

    @State var camera: MapViewCamera

    private let userLayers: [StyleLayerDefinition]

    public var speedLimit: Measurement<UnitSpeed>?
    public var speedLimitStyle: SpeedLimitView.SignageStyle?

    public var minimumSafeAreaInsets: EdgeInsets

    public var progressView: ((NavigationState?, (() -> Void)?) -> AnyView)?
    public var instructionsView: ((NavigationState?, Binding<Bool>, Binding<CGSize>) -> AnyView)?
    public var currentRoadNameView: ((NavigationState?) -> AnyView)?

    public init(
        ferrostarCore: FerrostarCore,
        styleURL: URL,
        camera: MapViewCamera = .automotiveNavigation(zoom: 17.0),
        minimumSafeAreaInsets: EdgeInsets = EdgeInsets(top: 16, leading: 16, bottom: 16, trailing: 16),
        @MapViewContentBuilder makeMapContent: () -> [StyleLayerDefinition] = { [] }
    ) {
        _ferrostarCore = StateObject(wrappedValue: ferrostarCore)
        self.styleURL = styleURL
        self.camera = camera
        self.minimumSafeAreaInsets = minimumSafeAreaInsets
        userLayers = makeMapContent()
    }

    public var body: some View {
        GeometryReader { geometry in
            ZStack {
                NavigationMapView(
                    styleURL: styleURL,
                    camera: $camera,
                    navigationState: ferrostarCore.state,
                    onStyleLoaded: { _ in camera = .automotiveNavigation(zoom: 17.0) }
                ) {
                    userLayers
                }
                .navigationMapViewContentInset(.landscape(within: geometry, horizontalPct: 0.65))
            }
        }
    }
}
