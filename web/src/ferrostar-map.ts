import { css, html, LitElement, PropertyValues, unsafeCSS } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import maplibregl, {
  GeolocateControl,
  LngLat,
  LngLatLike,
  Map,
} from "maplibre-gl";
import maplibreglStyles from "maplibre-gl/dist/maplibre-gl.css?inline";
import {
  NavigationController,
  RouteAdapter,
  TripState,
} from "@stadiamaps/ferrostar";
import "./instructions-view";
import "./trip-progress-view";
import { SimulatedLocationProvider } from "./location";
import CloseSvg from "./assets/directions/close.svg";

/**
 * A MapLibre-based map component specialized for navigation.
 */
@customElement("ferrostar-map")
export class FerrostarMap extends LitElement {
  @property()
  valhallaEndpointUrl: string = "";

  @property()
  styleUrl: string = "";

  @property()
  profile: string = "";

  @property()
  center: LngLatLike | null = null;

  @property()
  pitch: number = 60;

  @property()
  zoom: number = 6;

  @property({ attribute: false })
  httpClient?: Function = fetch;

  // TODO: type
  @property({ type: Object, attribute: false })
  locationProvider!: any;

  // TODO: type
  @property({ type: Object, attribute: false })
  options: object = {};

  @state()
  protected _tripState: TripState | null = null;

  /**
   * Configures the map on first load.
   *
   * Note: This will only be invoked if there is no map set
   * by the time of the first component update.
   * If you provide your own map parameter,
   * configuration is left to the caller.
   * Be sure to set the DOM parent via the slot as well.
   */
  @property({ type: Function, attribute: false })
  configureMap?: (map: Map) => void;

  @property({ type: Function, attribute: false })
  onNavigationStart?: (map: Map) => void;

  @property({ type: Function, attribute: false })
  onNavigationStop?: (map: Map) => void;

  @property({ type: Function, attribute: true })
  onTripStateChange?: (newState: TripState | null) => void;

  /**
   *  Styles to load which will apply inside the component
   *  (ex: for MapLibre plugins)
   */
  @property({ type: Object, attribute: false })
  customStyles?: object | null;

  /**
   * Enables voice guidance via the web speech synthesis API.
   * Defaults to false.
   */
  @property({ type: Boolean })
  useVoiceGuidance: boolean = false;

  /**
   * Automatically geolocates the user on map load.
   * Defaults to true.
   */
  @property({ type: Boolean })
  geolocateOnLoad: boolean = true;

  routeAdapter: RouteAdapter | null = null;

  /**
   * The MapLibre map instance.
   *
   * This will be automatically initialized by default
   * when the web component does its first update cycle.
   * However, you can also explicitly set this value
   * when initializing the web component to provide your own map instance.
   *
   * Note: If you set this property, you MUST also pass the map's container attribute
   * via the slot!
   */
  @property({ type: Object, attribute: false })
  map: maplibregl.Map | null = null;

  geolocateControl: GeolocateControl | null = null;
  navigationController: NavigationController | null = null;
  simulatedLocationMarker: maplibregl.Marker | null = null;
  lastSpokenUtteranceId: string | null = null;

  static styles = [
    unsafeCSS(maplibreglStyles),
    css`
      [hidden] {
        display: none !important;
      }

      #container {
        height: 100%;
        width: 100%;
      }

      #map,
      ::slotted(:first-child) {
        height: 100%;
        width: 100%;
        display: block;
      }

      instructions-view {
        top: 10px;
        position: absolute;
        left: 50%;
        transform: translateX(-50%);
        width: 80%;
        z-index: 1000;
      }

      #bottom-component {
        bottom: 10px;
        position: absolute;
        left: 50%;
        transform: translateX(-50%);
        max-width: 80%;
        z-index: 1000;
        display: flex;
        justify-content: space-between;
        gap: 10px;
      }

      #stop-button {
        display: flex;
        padding: 20px;
        background-color: white;
        border-radius: 50%;
        border: none;
        box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
        cursor: pointer;
        transition:
          background-color 0.3s,
          filter 0.3s;
      }

      #stop-button .icon {
        width: 20px;
        height: 20px;
      }

      #stop-button:hover {
        background-color: #e0e0e0;
      }
    `,
  ];

  constructor() {
    super();

    // A workaround for avoiding "Illegal invocation"
    if (this.httpClient === fetch) {
      this.httpClient = this.httpClient.bind(window);
    }
  }

  updated(changedProperties: PropertyValues<this>) {
    if (changedProperties.has("locationProvider") && this.locationProvider) {
      this.locationProvider.updateCallback = this.onLocationUpdated.bind(this);
    }
    if (this.map && this.map.loaded()) {
      if (changedProperties.has("styleUrl")) {
        this.map.setStyle(this.styleUrl);
      }
      if (changedProperties.has("center")) {
        if (changedProperties.get("center") === null && this.center !== null) {
          this.map.jumpTo({ center: this.center });
        } else if (this.center !== null) {
          if (
            this.map.getCenter().distanceTo(LngLat.convert(this.center)) >
            500_000
          ) {
            this.map.jumpTo({ center: this.center });
          } else {
            this.map.flyTo({ center: this.center });
          }
        }
      }
      if (changedProperties.has("pitch")) {
        this.map.setPitch(this.pitch);
      }
      if (changedProperties.has("zoom")) {
        this.map.setZoom(this.zoom);
      }
    }
  }

  firstUpdated() {
    // Skip initialization of the map if the user has supplied one already via a slot!
    const slotChildren =
      this.shadowRoot!.querySelector("slot")?.assignedElements() || [];
    if (slotChildren.length == 0 && this.map === null) {
      this.initMap();
    }
  }

  /**
   * Initialize the MapLibre Map control.
   *
   * This is run by default on firstUpdated,
   * but is skipped if the user adds a map of their own.
   */
  initMap() {
    this.map = new maplibregl.Map({
      container: this.shadowRoot!.getElementById("map")!,
      style: this.styleUrl
        ? this.styleUrl
        : "https://demotiles.maplibre.org/style.json",
      center: this.center ?? [0, 0],
      pitch: this.pitch,
      bearing: 0,
      zoom: this.zoom,
      attributionControl: { compact: true },
    });

    this.geolocateControl = new GeolocateControl({
      positionOptions: {
        enableHighAccuracy: true,
      },
      trackUserLocation: true,
    });

    this.map.addControl(this.geolocateControl);

    this.map.on("load", (e) => {
      if (this.geolocateOnLoad) {
        this.geolocateControl?.trigger();
      }

      if (this.configureMap !== undefined) {
        this.configureMap(e.target);
      }
    });
  }

  // TODO: type
  async getRoutes(initialLocation: any, waypoints: any) {
    // Initialize the route adapter
    // (NOTE: currently only supports Valhalla, but working toward expansion)
    this.routeAdapter = new RouteAdapter(
      this.valhallaEndpointUrl,
      this.profile,
      JSON.stringify(this.options),
    );

    // Generate the request body
    const routeRequest = this.routeAdapter.generateRequest(
      initialLocation,
      waypoints,
    );
    const method = routeRequest.get("method");
    let url = new URL(routeRequest.get("url"));
    const body = routeRequest.get("body");

    // Send the request to the Valhalla endpoint
    // FIXME: assert httpClient is not null
    const response = await this.httpClient!(url, {
      method: method,
      // FIXME: assert body is not null
      body: new Uint8Array(body).buffer,
    });

    const responseData = new Uint8Array(await response.arrayBuffer());
    try {
      return this.routeAdapter.parseResponse(responseData);
    } catch (e) {
      console.error("Error parsing route response:", e);
      throw e;
    }
  }

  // TODO: types
  startNavigation(route: any, config: any) {
    if (this.onNavigationStart && this.map) this.onNavigationStart(this.map);

    const navigationConfig = {
      waypointAdvance: config.waypointAdvance,
      stepAdvance: config.stepAdvance,
      routeDeviationTracking: config.routeDeviationTracking,
      routeRefreshStrategy: config.routeRefreshStrategy || "None",
      snappedLocationCourseFiltering: config.snappedLocationCourseFiltering,
    };

    this.navigationController = new NavigationController(
      route,
      navigationConfig,
    );

    const startingLocation = this.locationProvider.lastLocation
      ? {
          coordinates: this.locationProvider.lastLocation.coordinates,
          horizontalAccuracy:
            Number(this.locationProvider.lastLocation.horizontalAccuracy) ||
            0.0,
          courseOverGround: this.locationProvider.lastLocation.courseOverGround
            ? {
                degrees:
                  this.locationProvider.lastLocation.courseOverGround.degrees ||
                  0,
              }
            : null,
          timestamp: Date.now(),
          speed: this.locationProvider.lastLocation.speed
            ? { value: this.locationProvider.lastLocation.speed }
            : null,
        }
      : {
          coordinates: route.geometry[0],
          horizontalAccuracy: 0.0,
          courseOverGround: null,
          timestamp: Date.now(),
          speed: null,
        };

    try {
      const initialState =
        this.navigationController.getInitialState(startingLocation);
      this.tripStateUpdate(initialState);

      this.locationProvider.updateCallback = this.onLocationUpdated.bind(this);
      this.locationProvider.start();
    } catch (e) {
      console.error("Error getting initial state:", e);
    }

    // Update the UI with the initial trip state
    this.clearMap();

    this.map?.addSource("route", {
      type: "geojson",
      data: {
        type: "Feature",
        properties: {},
        geometry: {
          type: "LineString",
          coordinates: route.geometry.map(
            (point: { lat: number; lng: number }) => [point.lng, point.lat],
          ),
        },
      },
    });

    // TODO: Configuration param where to insert the layer
    this.map?.addLayer({
      id: "route",
      type: "line",
      source: "route",
      layout: {
        "line-join": "round",
        "line-cap": "round",
      },
      paint: {
        "line-color": "#3478f6",
        "line-width": 8,
      },
    });

    this.map?.addLayer(
      {
        id: "route-border",
        type: "line",
        source: "route",
        layout: {
          "line-join": "round",
          "line-cap": "round",
        },
        paint: {
          "line-color": "#FFFFFF",
          "line-width": 13,
        },
      },
      "route",
    );

    this.map?.setCenter(route.geometry[0]);

    if (this.locationProvider instanceof SimulatedLocationProvider) {
      this.simulatedLocationMarker = new maplibregl.Marker({
        color: "green",
      })
        .setLngLat(route.geometry[0])
        .addTo(this.map!);
    }
  }

  async stopNavigation() {
    // TODO: Factor out the UI layer from the core
    this.routeAdapter?.free();
    this.routeAdapter = null;
    this.navigationController?.free();
    this.navigationController = null;
    this.tripStateUpdate(null);
    this.clearMap();
    if (this.locationProvider) this.locationProvider.updateCallback = null;
    if (this.onNavigationStop && this.map) this.onNavigationStop(this.map);
  }

  private tripStateUpdate(newState: TripState | null) {
    this._tripState = newState;
    this.onTripStateChange?.(newState);
  }

  private onLocationUpdated() {
    if (!this.navigationController || !this._tripState) {
      return;
    }

    try {
      const formattedLocation = {
        coordinates: this.locationProvider.lastLocation.coordinates,
        horizontalAccuracy:
          Number(this.locationProvider.lastLocation.horizontalAccuracy) || 0.0,
        courseOverGround: this.locationProvider.lastLocation.courseOverGround
          ? {
              degrees:
                this.locationProvider.lastLocation.courseOverGround.degrees ||
                0,
            }
          : null,
        timestamp: Date.now(),
        speed: this.locationProvider.lastLocation.speed
          ? { value: this.locationProvider.lastLocation.speed }
          : null,
      };

      const tripStateCopy = JSON.parse(JSON.stringify(this._tripState));

      const newTripState = this.navigationController.updateUserLocation(
        formattedLocation,
        tripStateCopy,
      );

      this.tripStateUpdate(newTripState);

      // Update the simulated location marker if needed
      this.simulatedLocationMarker?.setLngLat(
        this.locationProvider.lastLocation.coordinates,
      );

      // Center the map on the user's location
      this.map?.easeTo({
        center: this.locationProvider.lastLocation.coordinates,
        bearing:
          this.locationProvider.lastLocation.courseOverGround.degrees || 0,
      });

      // Speak the next instruction if voice guidance is enabled
      const tripState = this._tripState;
      if (
        this.useVoiceGuidance &&
        tripState != null &&
        typeof tripState === "object"
      ) {
        if (
          "Navigating" in tripState &&
          tripState.Navigating?.spokenInstruction &&
          tripState.Navigating?.spokenInstruction.utteranceId !==
            this.lastSpokenUtteranceId
        ) {
          this.lastSpokenUtteranceId =
            tripState.Navigating?.spokenInstruction.utteranceId;
          window.speechSynthesis.cancel();
          window.speechSynthesis.speak(
            new SpeechSynthesisUtterance(
              tripState.Navigating?.spokenInstruction.text,
            ),
          );
        }
      }
    } catch (e) {
      console.error("Error updating location:", e);
    }
  }

  private clearMap() {
    this.map?.getLayer("route") && this.map?.removeLayer("route");
    this.map?.getLayer("route-border") && this.map?.removeLayer("route-border");
    this.map?.getSource("route") && this.map?.removeSource("route");
    this.simulatedLocationMarker?.remove();
  }

  render() {
    return html`
      <style>
        ${this.customStyles}
      </style>
      <div id="container">
        <div id="map">
          <!-- Fix names/ids; currently this is a breaking change -->
          <slot id="map-content"></slot>
          <div id="overlay">
            <instructions-view
              .tripState=${this._tripState}
            ></instructions-view>

            <div id="bottom-component">
              <trip-progress-view
                .tripState=${this._tripState}
              ></trip-progress-view>
              <button
                id="stop-button"
                @click=${this.stopNavigation}
                ?hidden=${!this._tripState}
              >
                <img src=${CloseSvg} alt="Stop navigation" class="icon" />
              </button>
            </div>
          </div>
        </div>
      </div>
    `;
  }
}
