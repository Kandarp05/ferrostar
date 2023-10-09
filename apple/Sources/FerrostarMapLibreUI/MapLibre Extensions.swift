import Foundation
import FerrostarCore
import MapLibre

extension FerrostarObservableState {
    var routePolyline: MLNPolyline {
        return MLNPolylineFeature(coordinates: fullRouteShape)
    }

    var remainingRoutePolyline: MLNPolyline {
        // FIXME
        return MLNPolylineFeature(coordinates: fullRouteShape)
    }
}
