/**
 * Pedestrian Simulation Model — Art Route
 * -------------------------------------------------------
 * Shapefiles neeed:
 *
 *   bid_vector.shp      – boundary polygon of the area
 *   art_route.shp       – polylines defining the walkable route
 *   studio_locations.shp – point locations (studios/POIs) agents visit
 *   bid_roads.shp       – roads (visual background)
 *   bid_water.shp       – water features (visual background)
 *   bid_buildings.shp   – buildings (visual background)
 *
 * Agent behaviour:
 *   1. Spawn at a random studio location
 *   2. Pick a random next studio, walk to it along the art route
 *   3. Pause for a random dwell time on arrival
 *   4. Repeat for a randomised number of visits (min_visits..max_visits)
 *   5. Disappear after completing all visits
 */

model art_spine_simulation

global {

    // Shapefiles 
    file boundary_file  <- file("../includes/bid_vector.shp");
    file route_file     <- file("../includes/art_spine.shp");
    file studios_file   <- file("../includes/studio_locations.shp");
    file roads_file     <- file("../includes/bid_roads.shp");
    file water_file     <- file("../includes/bid_water.shp");
    file buildings_file <- file("../includes/bid_buildings.shp");

    
    geometry shape <- envelope(boundary_file);

    // Parameters (for GUI)
    int   nb_pedestrians <- 100   min: 50    max: 200;
    float min_speed      <- 0.8   min: 0.1   max: 2.0;    // m/s
    float max_speed      <- 1.4   min: 0.5   max: 5.0;    // m/s
    float spawn_interval <- 2.0   min: 0.5   max: 10.0;   // seconds between spawns
    int   min_visits     <- 3     min: 1     max: 10;      // min studios per agent
    int   max_visits     <- 7     min: 2     max: 20;      // max studios per agent
    float min_dwell      <- 5.0   min: 0.5   max: 60.0;   // min pause at studio (s)
    float max_dwell      <- 20.0  min: 1.0   max: 120.0;  // max pause at studio (s)

   
    graph        route_network;
    list<point>  studio_points;
    geometry     boundary_geom;

    int  spawned_count <- 0;
    int  finished_count <- 0;
    bool all_spawned   <- false;
//initializing background layers and shapefiles

    init {
        // Background layers
        create water_layer     from: water_file;
        create buildings_layer from: buildings_file;
        create roads_layer     from: roads_file;

        // Boundary
        create boundary_layer from: boundary_file;
        boundary_geom <- union(boundary_layer collect each.shape);

        // Art route to a  navigable graph
        create route_layer from: route_file;
        route_network <- as_edge_graph(roads_layer);

        // Studio locations are a list of points
        create studio_layer from: studios_file;
        studio_points <- studio_layer collect (each.location);

        write "── Art Route Simulation initialised ────────────────────";
        write "  Route segments  : " + length(route_layer);
        write "  Route nodes     : " + length(route_network.vertices);
        write "  Studio locations: " + length(studio_points);
        write "  Pedestrians     : " + nb_pedestrians;
        write "  Visits / agent  : " + min_visits + " – " + max_visits;
        write "  Dwell time (s)  : " + min_dwell  + " – " + max_dwell;
        write "────────────────────────────────────────────────────────";

        if empty(studio_points) {
            write "⚠ WARNING: No locations found in studio_locations.shp.";
        }
        if length(route_network.vertices) = 0 {
            write "⚠ WARNING: Route graph is empty — check art_route.shp.";
        }
    }

    // gradually spawn pedestrians
    reflex spawn_pedestrians
        when: !all_spawned
          and !empty(studio_points)
          and (cycle mod max(1, int(spawn_interval / step)) = 0) {

        if spawned_count < nb_pedestrians {
            create pedestrian number: 1 {
                do initialise;
            }
            spawned_count <- spawned_count + 1;
        }
        if spawned_count >= nb_pedestrians {
            all_spawned <- true;
            write "All " + nb_pedestrians + " pedestrians spawned.";
        }
    }
}

// aspect of water
species water_layer {
    aspect default {
        draw shape color: rgb(164, 211, 238) border: rgb(100, 160, 200);
    }
}

//aspect of building
species buildings_layer {
    aspect default {
        draw shape color: rgb(210, 200, 190) border: rgb(150, 140, 130);
    }
}

// aspect of roads
species roads_layer {
    aspect default {
        draw shape color: rgb(200, 190, 175) width: 2;
    }
}

// aspect of outline
species boundary_layer {
    aspect default {
        draw shape color: rgb(0, 0, 0, 0) border: rgb(220, 50, 50) width: 3;
    }
}

// aspect of artspine
species route_layer {
    aspect default {
        draw shape color: rgb(80, 160, 80) width: 3;
    }
}

//aspect/draw of studio locations
species studio_layer {
    aspect default {
        // Outer ring
        draw circle(4.0) color: rgb(255, 200, 0, 130) border: rgb(200, 140, 0) width: 2;
        // Inner dot
        draw circle(1.8) color: #white;
    }
}

// pedestrians/agents
species pedestrian skills: [moving] {

    float        speed;
    path         current_path;

    // visits
    list<point>  visit_queue;       // remaining studios to visit
    point        current_target;    // studio currently heading toward
    int          visits_done  <- 0;
    int          total_visits;

    // dwell
    bool         dwelling  <- false;
    float        dwell_end <- 0.0;  // simulation time when dwell ends

    bool         done <- false;
    rgb          ped_color;

    // initializing agent
    action initialise {
        speed         <- min_speed + rnd(max_speed - min_speed);
        total_visits  <- min_visits + rnd(max_visits - min_visits);

        // randomize studios and take a subset for this agent's visit plan
        list<point> shuffled <- shuffle(copy(studio_points));
        visit_queue <- total_visits <= length(shuffled)
            ? shuffled[0::(total_visits - 1)]
            : shuffled;

        // spawn at first studio in the plan
        location       <- first(visit_queue);
        visit_queue    <- remove_duplicates(visit_queue - [first(visit_queue)]);
        current_target <- empty(visit_queue) ? nil : first(visit_queue);

        // path to first target
        if current_target != nil {
            current_path <- path_between(route_network, location, current_target);
            
        } else {
            done <- true;  // only 1 studio exists, nothing to do
        }

        // agent coloring randomized
        ped_color <- rgb(rnd(160, 255), rnd(50, 160), rnd(30, 110));
    }

    // the dwell at the studio
    reflex dwell when: !done and dwelling {
        if time >= dwell_end {
            dwelling    <- false;
            visits_done <- visits_done + 1;

            // remove visited studio 
            if !empty(visit_queue) {
                visit_queue <- visit_queue - [first(visit_queue)];
            }
            current_target <- empty(visit_queue) ? nil : first(visit_queue);

            if current_target = nil {
                // if visit all, agent complete
                done            <- true;
                finished_count  <- finished_count + 1;
            } else {
                // taking path to next studio
                current_path <- path_between(route_network, location, current_target);
            }
        }
    }

    // walk to the next studio
    reflex walk when: !done and !dwelling and current_target != nil {
        //do follow path: current_path;
       //do goto target: current_target;
       do goto target: current_target on:route_network;
        

        // if close enough to target- start dwelling
        if location distance_to current_target < 2.0 {
            location  <- current_target;   // snap to exact point
            dwelling  <- true;
            dwell_end <- time + min_dwell + rnd(max_dwell - min_dwell);
        }
    }

    // remove agent/ped when all visits done
    reflex disappear when: done {
        do die;
    }

    // visualization
    aspect default {
        // agent body
        draw circle(10) color: ped_color border: #white;

        // ring indicated when dwelling
        if dwelling {
            draw circle(40)
                color: rgb(ped_color.red, ped_color.green, ped_color.blue, 60)
                border: ped_color width: 10;
        }

        // direction arrow while walking
        if !dwelling and current_target != nil {
            draw polyline([
                location,
                { location.x + cos(heading) * 3.0,
                  location.y + sin(heading) * 3.0 }
            ]) color: #white width: 5;
        }
    }
}

// experiment 
experiment "Art Route Simulation" type: gui {

    parameter "Number of pedestrians" var: nb_pedestrians
        min: 50  max: 200  step: 10;
    parameter "Min speed (m/s)"       var: min_speed
        min: 0.1 max: 2.0  step: 0.1;
    parameter "Max speed (m/s)"       var: max_speed
        min: 0.5 max: 5.0  step: 0.1;
    parameter "Spawn interval (s)"    var: spawn_interval
        min: 0.5 max: 10.0 step: 0.5;
    parameter "Min studios to visit"  var: min_visits
        min: 1   max: 10   step: 1;
    parameter "Max studios to visit"  var: max_visits
        min: 2   max: 20   step: 1;
    parameter "Min dwell time (s)"    var: min_dwell
        min: 0.5 max: 60.0 step: 0.5;
    parameter "Max dwell time (s)"    var: max_dwell
        min: 1.0 max: 120.0 step: 1.0;

    output {

        // main map display
        display "Art Route Map" type: 2d background: rgb(240, 235, 225) {
            species water_layer     aspect: default;
            species buildings_layer aspect: default;
            species roads_layer     aspect: default;
            species route_layer     aspect: default;
            species studio_layer    aspect: default;
            species boundary_layer  aspect: default;
            species pedestrian      aspect: default;

            overlay position: {5, 5} size: {250, 115}
                    background: #black transparency: 0.45 {
                draw "Spawned   : " + spawned_count
                    color: #white  font: font("Arial", 13, #bold) at: {10, 22};
                draw "Walking   : " + length(pedestrian where !each.dwelling)
                    color: #yellow font: font("Arial", 13, #bold) at: {10, 44};
                draw "At studio : " + length(pedestrian where each.dwelling)
                    color: #orange font: font("Arial", 13, #bold) at: {10, 66};
                draw "Finished  : " + finished_count
                    color: #lime   font: font("Arial", 13, #bold) at: {10, 88};
            }
        }

        // chart
        display "Agent Status Chart" type: 2d {
            chart "Pedestrian activity over time" type: series background: #white {
                data "Walking"   value: length(pedestrian where !each.dwelling)
                    color: #orange style: line;
                data "At studio" value: length(pedestrian where each.dwelling)
                    color: #blue   style: line;
                data "Finished"  value: finished_count
                    color: #green  style: line;
            }
        }

        // monitorization
        monitor "Spawned"    value: spawned_count;
        monitor "Walking"    value: length(pedestrian where !each.dwelling);
        monitor "At studio"  value: length(pedestrian where each.dwelling);
        monitor "Finished"   value: finished_count;
        monitor "Cycle"      value: cycle;
    }
}
