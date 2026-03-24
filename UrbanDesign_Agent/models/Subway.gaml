/**
 * Subway Commuter + Art Route Pedestrian Simulation
 * -------------------------------------------------------
 * Extends the Art Route model with subway commuters emerging
 * from Carroll Street and Union Street stations, walking the
 * road network to random buildings in the BID.
 *
 * Shapefiles (includes/ folder):
 *   bid_vector.shp        – boundary polygon
 *   art_spine.shp         – art route polylines
 *   studio_locations.shp  – studio/POI points
 *   bid_roads.shp         – road network polylines
 *   bid_water.shp         – water features
 *   bid_buildings.shp     – BID-area buildings
 *   subway.shp            – extended buildings near subway stops
 */

model Subway

global {

    // ── Shapefiles ──────────────────────────────────────────────────
    file boundary_file         <- file("../includes/bid_vector.shp");
    file route_file            <- file("../includes/art_spine.shp");
    file studios_file          <- file("../includes/studio_locations.shp");
    file roads_file            <- file("../includes/bid_roads.shp");
    file water_file            <- file("../includes/bid_water.shp");
    file buildings_file        <- file("../includes/bid_buildings.shp");
    file subway_buildings_file <- file("../includes/subway.shp");

    // ── World envelope ──────────────────────────────────────────────
    geometry shape <- envelope(boundary_file);

    // ── Subway station locations (converted from WGS84 in init) ─────
    point carroll_st_loc;
    point union_st_loc;

    // ── Art-route pedestrian parameters (GUI sliders) ───────────────
    int   nb_pedestrians <- 100   min: 50    max: 200;
    float min_speed      <- 0.8   min: 0.1   max: 2.0;    // m/s
    float max_speed      <- 1.4   min: 0.5   max: 5.0;    // m/s
    float spawn_interval <- 2.0   min: 0.5   max: 10.0;   // seconds between spawns
    int   min_visits     <- 3     min: 1     max: 10;
    int   max_visits     <- 7     min: 2     max: 20;
    float min_dwell      <- 5.0   min: 0.5   max: 60.0;   // seconds
    float max_dwell      <- 20.0  min: 1.0   max: 120.0;  // seconds

    // ── Commuter parameters (GUI sliders) ───────────────────────────
    int   nb_commuters             <- 150  min: 0    max: 500;
    float commuter_min_speed       <- 1.0  min: 0.3  max: 3.0;   // m/s
    float commuter_max_speed       <- 1.8  min: 0.5  max: 5.0;   // m/s
    float commuter_spawn_interval  <- 1.5  min: 0.5  max: 10.0;  // seconds
    float art_exposure_radius      <- 50.0 min: 10.0 max: 200.0; // meters from art spine
    float rush_hour_peak           <- 3.0  min: 1.0  max: 5.0;   // peak multiplier

    // ── Internal state ──────────────────────────────────────────────
    graph        route_network;    // art spine graph (pedestrians)
    graph        road_network;     // bid roads graph (commuters)
    list<point>  studio_points;
    list<point>  building_points;  // commuter destinations (building centroids)
    geometry     boundary_geom;
    geometry     art_spine_geom;   // union of art route for proximity checks

    // Pedestrian counters
    int  ped_spawned     <- 0;
    int  ped_finished    <- 0;
    bool ped_all_spawned <- false;

    // Commuter counters
    int  com_spawned     <- 0;
    int  com_finished    <- 0;

    // Art spine exposure tracking
    int  commuters_near_art  <- 0;  // current count near art spine
    int  total_art_exposures <- 0;  // cumulative exposure events

    init {
        // Background layers
        create water_layer            from: water_file;
        create buildings_layer        from: buildings_file;
        create subway_buildings_layer from: subway_buildings_file;
        create roads_layer            from: roads_file;

        // Boundary
        create boundary_layer from: boundary_file;
        boundary_geom <- union(boundary_layer collect each.shape);

        // Art route → navigable graph (for pedestrians) + geometry for exposure checks
        create route_layer from: route_file;
        route_network <- as_edge_graph(route_layer);
        art_spine_geom <- union(route_layer collect each.shape);

        // Road network → navigable graph (for commuters)
        road_network <- as_edge_graph(roads_layer);

        // Studio locations
        create studio_layer from: studios_file;
        studio_points <- studio_layer collect (each.location);

        // Building centroids as commuter destinations
        building_points <- buildings_layer collect (each.location);

        // Convert WGS84 station coords → GAMA internal CRS
        carroll_st_loc <- (to_GAMA_CRS({-73.99438515000179, 40.68110765274973}, "EPSG:4326")).location;
        union_st_loc   <- (to_GAMA_CRS({-73.98304934187288, 40.67720622858433}, "EPSG:4326")).location;

        write "Carroll St (projected): " + carroll_st_loc;
        write "Union St (projected):   " + union_st_loc;
        write "First studio for ref:   " + first(studio_points);

        // Station markers (visual only)
        create station_marker number: 1 {
            location     <- carroll_st_loc;
            station_name <- "Carroll St";
        }
        create station_marker number: 1 {
            location     <- union_st_loc;
            station_name <- "Union St";
        }

        write "── Subway + Art Route Simulation ─────────────────────────";
        write "  Route segments   : " + length(route_layer);
        write "  Road segments    : " + length(roads_layer);
        write "  Road graph nodes : " + length(road_network.vertices);
        write "  Studio locations : " + length(studio_points);
        write "  Building targets : " + length(building_points);
        write "  Art pedestrians  : " + nb_pedestrians;
        write "  Subway commuters : " + nb_commuters;
        write "──────────────────────────────────────────────────────────";

        if empty(studio_points) {
            write "WARNING: No studio locations found.";
        }
        if empty(building_points) {
            write "WARNING: No building destinations found.";
        }
        if length(road_network.vertices) = 0 {
            write "WARNING: Road graph is empty — check bid_roads.shp.";
        }
    }

    // ── Spawn art-route pedestrians ─────────────────────────────────
    reflex spawn_pedestrians
        when: !ped_all_spawned
          and !empty(studio_points)
          and (cycle mod max(1, int(spawn_interval / step)) = 0) {

        if ped_spawned < nb_pedestrians {
            create pedestrian number: 1 {
                do initialise;
            }
            ped_spawned <- ped_spawned + 1;
        }
        if ped_spawned >= nb_pedestrians {
            ped_all_spawned <- true;
            write "All " + nb_pedestrians + " art-route pedestrians spawned.";
        }
    }

    // ── Spawn subway commuters (continuous with rush-hour curve) ────
    reflex spawn_commuters
        when: !empty(building_points)
          and (cycle mod max(1, int(commuter_spawn_interval / step)) = 0) {

        // Rush-hour multiplier: sine wave peaks at cycle ~500 and ~1500
        // Ranges from 1.0 (off-peak) to rush_hour_peak (peak)
        float rush_mult <- 1.0 + (rush_hour_peak - 1.0) * max(0.0, sin(cycle / 300.0));
        int target_pop <- min(500, int(nb_commuters * rush_mult));

        if length(commuter) < target_pop {
            create commuter number: 1 {
                do initialise;
            }
            com_spawned <- com_spawned + 1;
        }
    }

    // ── Track commuter exposure to art spine ────────────────────────
    reflex track_art_exposure when: art_spine_geom != nil {
        commuters_near_art <- length(commuter where (each.location distance_to art_spine_geom < art_exposure_radius));
        total_art_exposures <- total_art_exposures + commuters_near_art;
    }

    // ── Periodic debug summary ──────────────────────────────────────
    reflex debug_status when: cycle mod 200 = 0 and cycle > 0 {
        write "[STATUS] cycle=" + cycle
            + " commuters=" + length(commuter)
            + " arrived=" + com_finished
            + " near_art=" + commuters_near_art
            + " total_exposure=" + total_art_exposures;
    }
}

// ── Background: water ───────────────────────────────────────────────
species water_layer {
    aspect default {
        draw shape color: rgb(164, 211, 238) border: rgb(100, 160, 200);
    }
}

// ── Background: BID buildings ───────────────────────────────────────
species buildings_layer {
    aspect default {
        draw shape color: rgb(210, 200, 190) border: rgb(150, 140, 130);
    }
}

// ── Background: extended subway-area buildings ──────────────────────
species subway_buildings_layer {
    aspect default {
        draw shape color: rgb(200, 195, 185) border: rgb(160, 155, 145);
    }
}

// ── Background: roads ───────────────────────────────────────────────
species roads_layer {
    aspect default {
        draw shape color: rgb(200, 190, 175) width: 2;
    }
}

// ── Boundary outline ────────────────────────────────────────────────
species boundary_layer {
    aspect default {
        draw shape color: rgb(0, 0, 0, 0) border: rgb(220, 50, 50) width: 3;
    }
}

// ── Art route lines ─────────────────────────────────────────────────
species route_layer {
    aspect default {
        draw shape color: rgb(80, 160, 80) width: 3;
    }
}

// ── Studio location markers ─────────────────────────────────────────
species studio_layer {
    aspect default {
        draw circle(4.0) color: rgb(255, 200, 0, 180) border: rgb(200, 140, 0) width: 2;
        draw circle(1.8) color: #white;
    }
}

// ── Subway station markers (same pattern as studio_layer) ───────────
species station_marker {
    string station_name;
    aspect default {
        draw circle(6.0) color: rgb(255, 0, 80) border: rgb(180, 0, 50) width: 2;
        draw circle(2.5) color: #white;
    }
}

// ── Art-route pedestrian ────────────────────────────────────────────
species pedestrian skills: [moving] {

    float        speed;
    path         current_path;
    list<point>  visit_queue;
    point        current_target;
    int          visits_done  <- 0;
    int          total_visits;
    bool         dwelling  <- false;
    float        dwell_end <- 0.0;
    bool         done <- false;
    rgb          ped_color;

    action initialise {
        speed        <- min_speed + rnd(max_speed - min_speed);
        total_visits <- min_visits + rnd(max_visits - min_visits);

        list<point> shuffled <- shuffle(copy(studio_points));
        visit_queue <- total_visits <= length(shuffled)
            ? shuffled[0::(total_visits - 1)]
            : shuffled;

        location       <- first(visit_queue);
        visit_queue    <- remove_duplicates(visit_queue - [first(visit_queue)]);
        current_target <- empty(visit_queue) ? nil : first(visit_queue);

        if current_target != nil {
            current_path <- path_between(route_network, location, current_target);
        } else {
            done <- true;
        }

        ped_color <- rgb(rnd(160, 255), rnd(50, 160), rnd(30, 110));
    }

    reflex dwell when: !done and dwelling {
        if time >= dwell_end {
            dwelling    <- false;
            visits_done <- visits_done + 1;

            if !empty(visit_queue) {
                visit_queue <- visit_queue - [first(visit_queue)];
            }
            current_target <- empty(visit_queue) ? nil : first(visit_queue);

            if current_target = nil {
                done         <- true;
                ped_finished <- ped_finished + 1;
            } else {
                current_path <- path_between(route_network, location, current_target);
            }
        }
    }

    reflex walk when: !done and !dwelling and current_target != nil {
        do goto target: current_target on: route_network;

        if location distance_to current_target < 2.0 {
            location  <- current_target;
            dwelling  <- true;
            dwell_end <- time + min_dwell + rnd(max_dwell - min_dwell);
        }
    }

    reflex disappear when: done {
        do die;
    }

    aspect default {
        draw circle(5) color: ped_color border: #white;

        if dwelling {
            draw circle(4.0)
                color: rgb(ped_color.red, ped_color.green, ped_color.blue, 80)
                border: ped_color width: 1;
        }

        if !dwelling and current_target != nil {
            draw polyline([
                location,
                { location.x + cos(heading) * 3.0,
                  location.y + sin(heading) * 3.0 }
            ]) color: #white width: 1;
        }
    }
}

// ── Subway commuter ─────────────────────────────────────────────────
species commuter skills: [moving] {

    float  speed;
    point  target_building;
    rgb    com_color <- #red;

    action initialise {
        // Spawn at a random station (50/50 split)
        point station <- flip(0.5) ? carroll_st_loc : union_st_loc;
        location <- station;
        // Pick a random building as destination
        target_building <- one_of(building_points);
        // Walking speed
        speed <- commuter_min_speed + rnd(commuter_max_speed - commuter_min_speed);
        // Bright cyan/blue — distinct from warm-toned pedestrians
        com_color <- rgb(0, rnd(180, 255), rnd(220, 255));

    }

    reflex walk when: target_building != nil {
        do goto target: target_building;

        if location distance_to target_building < 5.0 {
            com_finished <- com_finished + 1;
            do die;
        }
    }

    aspect default {
        draw circle(5) color: rgb(0, 200, 255) border: rgb(0, 80, 150);
    }
}

// ── Experiment ──────────────────────────────────────────────────────
experiment "Subway + Art Route Simulation" type: gui {

    // Art-route parameters
    parameter "Art pedestrians"       var: nb_pedestrians    min: 50   max: 200   step: 10;
    parameter "Art min speed (m/s)"   var: min_speed         min: 0.1  max: 2.0   step: 0.1;
    parameter "Art max speed (m/s)"   var: max_speed         min: 0.5  max: 5.0   step: 0.1;
    parameter "Art spawn interval"    var: spawn_interval    min: 0.5  max: 10.0  step: 0.5;
    parameter "Art min studios"       var: min_visits        min: 1    max: 10    step: 1;
    parameter "Art max studios"       var: max_visits        min: 2    max: 20    step: 1;
    parameter "Art min dwell (s)"     var: min_dwell         min: 0.5  max: 60.0  step: 0.5;
    parameter "Art max dwell (s)"     var: max_dwell         min: 1.0  max: 120.0 step: 1.0;

    // Commuter parameters
    parameter "Subway commuters"        var: nb_commuters            min: 0    max: 500  step: 10;
    parameter "Commuter min speed"      var: commuter_min_speed      min: 0.3  max: 3.0  step: 0.1;
    parameter "Commuter max speed"      var: commuter_max_speed      min: 0.5  max: 5.0  step: 0.1;
    parameter "Commuter spawn interval" var: commuter_spawn_interval min: 0.5  max: 10.0 step: 0.5;
    parameter "Rush hour peak mult"    var: rush_hour_peak          min: 1.0  max: 5.0  step: 0.5;
    parameter "Art exposure radius (m)" var: art_exposure_radius    min: 10.0 max: 200.0 step: 10.0;

    output {

        // ── Main 2D map ─────────────────────────────────────────
        display "Subway & Art Route Map" type: 2d background: rgb(240, 235, 225) {
            species water_layer            aspect: default;
            species subway_buildings_layer aspect: default;
            species buildings_layer        aspect: default;
            species roads_layer            aspect: default;
            species route_layer            aspect: default;
            species studio_layer           aspect: default;
            species boundary_layer         aspect: default;
            species station_marker         aspect: default;
            species pedestrian             aspect: default;
            species commuter               aspect: default;

            overlay position: {5, 5} size: {280, 160}
                    background: #black transparency: 0.45 {
                draw "-- Art Route Pedestrians --"
                    color: #white  font: font("Arial", 12, #bold) at: {10, 18};
                draw "  Spawned: " + ped_spawned + "  Finished: " + ped_finished
                    color: #orange font: font("Arial", 12, #plain) at: {10, 36};
                draw "  Walking: " + length(pedestrian where !each.dwelling)
                  + "  At studio: " + length(pedestrian where each.dwelling)
                    color: #yellow font: font("Arial", 12, #plain) at: {10, 54};
                draw "-- Subway Commuters --"
                    color: #white  font: font("Arial", 12, #bold) at: {10, 78};
                draw "  Spawned: " + com_spawned + "  Finished: " + com_finished
                    color: rgb(100, 170, 255) font: font("Arial", 12, #plain) at: {10, 96};
                draw "  Walking: " + length(commuter)
                    color: rgb(100, 170, 255) font: font("Arial", 12, #plain) at: {10, 114};
                draw "Cycle: " + cycle
                    color: #white font: font("Arial", 11, #plain) at: {10, 138};
            }
        }

        // ── Live chart ──────────────────────────────────────────
        display "Agent Activity" type: 2d {
            chart "Activity over time" type: series background: #white {
                data "Art walking"      value: length(pedestrian where !each.dwelling)
                    color: #orange style: line;
                data "Art at studio"    value: length(pedestrian where each.dwelling)
                    color: #green  style: line;
                data "Art finished"     value: ped_finished
                    color: rgb(180, 120, 60) style: line;
                data "Commuters walking" value: length(commuter)
                    color: rgb(60, 130, 255) style: line;
                data "Commuters near art" value: commuters_near_art
                    color: #magenta style: line;
            }
        }

        // ── Exposure chart ─────────────────────────────────────
        display "Art Spine Exposure" type: 2d {
            chart "Commuter exposure to art spine" type: series background: #white {
                data "Commuters near art spine" value: commuters_near_art
                    color: #magenta style: line;
                data "Cumulative exposures (÷10)" value: total_art_exposures / 10
                    color: rgb(150, 0, 150) style: line;
            }
        }

        // ── Monitors ────────────────────────────────────────────
        monitor "Art Spawned"         value: ped_spawned;
        monitor "Art Walking"         value: length(pedestrian where !each.dwelling);
        monitor "Art At Studio"       value: length(pedestrian where each.dwelling);
        monitor "Art Finished"        value: ped_finished;
        monitor "Commuters Spawned"   value: com_spawned;
        monitor "Commuters Walking"   value: length(commuter);
        monitor "Commuters Arrived"   value: com_finished;
        monitor "Near Art Spine"      value: commuters_near_art;
        monitor "Total Art Exposures" value: total_art_exposures;
        monitor "Cycle"               value: cycle;
    }
}