#include "traffic/SimulationView.hpp"
#include "imgui.h"
#include <algorithm>
#include <cctype>
#include <string>

namespace traffic {
namespace {
struct Chapter {const char* title;const char* body;};
constexpr Chapter chapters[]{
    {"Getting started",
     "1. Open a PNG in File > Open map PNG. Set its meters-per-pixel value in Files.\n\n"
     "2. Open a .nodes network, or use Map design to place and connect road nodes. Mark spawn nodes and trip destinations in their node popups.\n\n"
     "3. Open Native simulation or SUMO simulation and click Use Native / Use SUMO. The toolbar always names the engine that will run. Switching engines resets vehicles and counters, preserves the design and profiles, and pauses. Browsing another section does not switch engines.\n\n"
     "4. Press Play, or choose Manual steps and press Step. Reset traffic restarts the current engine with the current seed.\n\n"
     "Use the common map and demand settings to compare both implementations. They share the design and vehicle profiles, but do not run two traffic populations at once."},
    {"Choosing an engine",
     "NATIVE SIMULATION\nThe previous C++ simulation runs lanes, lane changes, routing, collisions, signals and free junctions. Native driving, recovery and advanced controls are in its own section. Predictive junctions and collision-safe following remain enabled. Sublane filtering, smooth mixed traffic, RTS refresh, tug assistance, turn negotiation, fleet recovery, deletion policies and priority swapping default to off; enable them only when wanted. RTS, tug and negotiation controls belong to this implementation; some are bypassed by predictive movement.\n\n"
     "SUMO SIMULATION\nActual headless Eclipse SUMO controls lane traffic, Krauss following, SL2015 lane changes and sublanes, guided intersections, route selection and insertion safety. Explicit open junctions still use the native free-space controller through a handoff. No SUMO GUI is opened.\n\n"
     "SUMO must be installed in build/sumo-runtime/sumo, beside the executable, or located by SUMO_HOME. A missing runtime shows an error; it never silently selects Native. Use scripts/install-sumo.ps1 if needed. Engine diagnostics contains the actual version and session-log path.\n\n"
     "IMPORT DIFFERENCES\nSUMO currently uses each lane's minimum authored width for tapers. Native lane-change restriction sections are inactive in SUMO. Painted stop lines select SUMO junction approaches; the actual lane stop position is its generated junction boundary. Native turn controls apply to free areas; SUMO chooses guided turn speeds."},
    {"Clock and spawning",
     "Real time advances with the wall clock; Time scale changes the playback rate. Native uses 120 Hz outer ticks; SUMO uses 0.1-second steps with smaller native free-motion substeps.\n\n"
     "Manual steps advances only when Step is pressed. Type the requested duration; the work is processed across frames. Cancel remaining step discards only unfinished time. Pause adds no simulated time.\n\n"
     "Each spawn node has its own Vehicles / s rate. Automatic spawning requests vehicles from those nodes. The Vehicle limit includes SUMO requests waiting off-network. Existing vehicles are not deleted to enforce that limit. Spawn vehicle requests one extra vehicle from a marked source.\n\n"
     "Seed is applied on reset. Reset traffic clears vehicles and statistics, keeping the map, nodes, lights and profiles. Engine switching does the same and leaves the simulation paused. Both engines use random reachable destinations, subject to the source's destination policy and feasible routes. SUMO computes the fastest route to the selected destination.\n\n"
     "Type numeric settings and press Enter where required to commit them."},
    {"Maps and files",
     "File provides New map, Open map PNG, Open node network, Save node network, Save as and Export map PNG. PNG and .nodes files are saved separately. Keep their .signals companions with them; those files contain light identities, timings, permissions and detectors.\n\n"
     "Meters per pixel sets the physical map scale. Node coordinates and vehicle dimensions use meters. A scale change changes the mask's physical size, not every existing node coordinate.\n\n"
     "PNG COLORS\nRed: road mask. Black: blocked space. Blue: traffic-light markings. Pink: solid native mask divider. Legacy green and yellow markings do not define trip endpoints or spawning: use node settings.\n\n"
     "Paint / erase by dragging. Line tools use two clicks. Editing geometry pauses or resets traffic; save before replacing a map. The Native controller uses the road mask for body clearance. SUMO road traffic follows its generated lane geometry; the PNG remains the free-junction mask and drawing background.\n\n"
     "Navigation: wheel to zoom, middle-button drag to pan, Fit view to restore the overview."},
    {"Nodes and connections",
     "Place nodes adds isolated points on the road. It does not automatically connect them or enable spawning. Fixed-size N-number badges identify nodes above lights and traffic, with a leader to the actual node. Left-click the node marker to open its properties. The popup groups settings into General, Roads, Lane exits, Signals and Junction tabs.\n\n"
     "Mark Spawn point and set Vehicles / s for an independent source. Mark Trip destination where trips may finish. Exclude this node from its own spawn trips prevents that source's new vehicles from selecting it as their destination; other origins may still arrive there.\n\n"
     "In the Roads tab choose the new connection type and lane count, then Connect to another node starts a pending connection. Click the target node to commit it. Sever a connection similarly requires a target. Existing nodes remain in place.\n\n"
     "CANCEL A TOOL\nThe bottom-left map bar displays the active action. Click Cancel (Esc), or press Escape, to cancel a pending connection, sever operation, road sketch, node insertion or drawing tool. It discards unfinished work and keeps committed nodes and roads. Clicking Cancel never adds a node or paints behind the button.\n\n"
     "LANE DESTINATIONS\nOpen the Lane exits tab on the road's exit node. Choose the incoming road and a lane. Lane 1 is leftmost when facing travel; a two-way road has separate 1-n numbering in each direction. Up to 32 lanes are supported per direction.\n\n"
     "Manual lane exits are required before Play, Step or manual spawning. Destination endpoints need no continuing exit. A notification lists every missing node/road/lane. In Lane exits choose Edit lane connections on map to open the sidebar editor. Select the incoming road and click one source lane number. Gold marks that incoming lane. Click outgoing lanes on the map, or select an outgoing road and toggle numbered target-lane buttons. Teal ribbons and arrows show only the chosen source lane's exits. Hover a lane to identify its entry node, travel direction and lane number. Lane 1 is leftmost facing travel, independently in each direction. No node-ID lists are needed.\n\n"
     "Apply lane exits commits the selected source lane and resets traffic. Apply or Discard changes before switching the source lane; each lane has its own preview. Done returns to map tools. Escape closes the editor and discards unfinished changes. Movement priorities (optional) assigns a priority to each chosen outgoing lane. Links can cross free red space between any existing lanes, even with no shared junction. The map mask and vehicle size still determine whether the crossing is physically possible.\n\n"
     "An empty exit list or an unconfigured non-destination lane blocks simulation. Each selected target lane has its own priority: 1 is highest, larger numbers yield to smaller numbers on conflicting paths inside the free junction. Save the network to retain priorities. Removing a road/lane removes obsolete references. These junction rules apply in Native and SUMO, including free-junction entry/exit choices.\n\n"
     "File > Examples > Seven-lane routing loads the requested example: lane 1 turns left into lane 1, lanes 2-6 go straight into lanes 1-7, and lane 7 turns right into the outermost outgoing lane.\n\n"
     "Ctrl + left-click connected nodes in travel order to select a path for road properties. Ordinary node clicks open settings instead."},
    {"Roads and sections",
     "MAP DESIGN WORKFLOW\n1. Paint drivable area: drag Paint road area over blocked space. Use Erase area to trim it. The colored area is the road surface, not a traffic network.\n\n"
     "2. Create roads: choose direction, lanes per direction, lane width and speed. Sketch road adds points inside that surface; at least two points are needed to Finish road. The endpoints become connected nodes. A one-way road runs from the first point to the last. Taper and point widths holds optional width adjustments.\n\n"
     "3. Configure nodes and trips: click nodes to mark Spawn point and Trip destination in General. Set each incoming lane's permitted movements in Lane exits before Play. Add junction or signal nodes on an existing road when needed.\n\n"
     "Select / inspect or Escape stops the current tool. Choosing another tool discards unfinished work. Advanced road properties holds path editing; Advanced markings holds line tools. Save the PNG and node network separately through File, together with their .signals companions.\n\n"
     "Sketch road places centerline points and previews the next segment under the cursor. Choose direction, lane count and the next point's width before clicking. Finish road commits the sketch; Undo last point removes an unfinished point. Cancel discards the whole uncommitted sketch. For an individual lane, choose One-way road with one lane.\n\n"
     "One-way roads retain their direction. Two-way roads have independent forward/reverse lanes and widths. Connect a third arm for a T-junction or a fourth for a crossroad; straight-through movements remain available. Insert junction node on road splits an existing connection for another arm.\n\n"
     "NATIVE GEOMETRY\nWidths at successive points form tapering ribbons. Native lane-change sections use meter distances from the authored from-node. Select direction and lane (0 = all), choose Allow entering / Allow leaving, then Add section. Restrictions cover a vehicle's lane-change maneuver, not straight travel.\n\n"
     "A two-way road can have an optional U-turn opening while retaining straight routes. A node can also allow guided U-turn connections.\n\n"
     "Mark This node is a turn only when it should apply custom turn braking, then set its radius and deceleration. Unmarked shape/width points do not impose that custom slowdown.\n\n"
     "SUMO GEOMETRY\nIts current importer uses constant minimum lane widths, generated turn geometry and normal SUMO lane-changing rules. Native per-position restriction sections are not imported. These are engine differences, not disabled Native features."},
    {"Free junctions",
     "A positive open-junction radius ends the predetermined road lanes at the free area. Vehicles navigate the red mask toward an outgoing lane, rather than finishing their trips at the entry. The destination remains a marked trip node. A zero radius uses guided junction connections.\n\n"
     "Native handles these areas directly. SUMO hands a vehicle to the native controller, preserving its ID, sampled profile and destination. The incoming lane is guarded until the rear clears; it rejoins SUMO at its actual exit position. A handoff is not an arrival.\n\n"
     "SUMO checks feasible native crossings before departure. Handoff approaches stabilize their existing sublane, without forcing all vehicles into its center. Committed intersecting free-space corridors are reserved conservatively; disjoint crossings may proceed together.\n\n"
     "NATIVE PATH PRIORITIES\nConfigure each movement in Lane exits: incoming lane, outgoing node, target lane, priority. Conflicting lower-priority traffic yields; nonconflicting paths proceed. Vehicles approaching a red stop do not reserve priority over free traffic. In Native simulation > Junction priority, Swap priority after a long yield grants a waiting vehicle temporary precedence after the typed timeout. It defaults to off. Vehicles physically in its way must clear before other conflicts yield; collision checks stay active.\n\nThe two models have different capacity and recovery behavior. Geometry changes reset traffic. The separate stationary-open-junction timeout and deletion counter are available in Monitoring."},
    {"Signals and detectors",
     "ROAD-ALIGNED SIGNALS\nIn Map design choose Place signal node on road. The longitudinal reference lines show where to click. A road-station node is inserted there, preserving lane continuity and existing downstream rules. Click the node, open Signals, select its OUT approach and Create perpendicular light. The stop line spans the road at a right angle. Draw allowed exit then starts that light's permission-gate tool.\n\n"
     "ALL SIGNALS ARE DIRECTIONAL\nThe OUT arrow is controlled by red/yellow/green. The reverse IN arrow passes this light regardless of color, while still respecting other vehicles. Reverse IN / OUT reverses the controlled direction. Two arrows sit on the right-hand side of their respective travel directions, with IN and OUT printed directly on their shafts. OUT obeys the light; IN passes it. The signal sidebar separates Travel direction, Timing, Allowed exit and Queue detectors. Existing signal files infer an initial OUT normal from their line endpoints; review the arrows and flip if needed. The direction and road-node anchor are saved with the light.\n\n"
     "Draw a stop line using the traffic-light line tool. For a permissive light, draw its stop line first and then its linked allowed-exit line across the outgoing movement. Select a light to edit green, yellow, red and offset seconds while running. Duration changes use the current simulation clock and can change the phase immediately.\n\n"
     "A vehicle whose planned movement crosses the linked allowed exit after the stop line may pass through red/yellow, while still yielding to traffic. The exit is a permission gate, never a trip destination. Every light keeps its own linked gate.\n\n"
     "SUMO signals operate at generated junction stops. Native queue-planning controls appear only when Native is active. Both use the common light timing and drawing tools.\n\n"
     "QUEUE DETECTORS\nEach light can have multiple detector lines. Draw them nearest to farthest upstream. A line is occupied whenever any rendered body touches it, moving or stopped. The UI shows YES / NO and occupancy percentage.\n\n"
     "Quality is the highest numbered line whose occupancy reaches the chosen threshold over a full rolling observation window. Detector edits restart changed observation windows. Detectors do not stop or delete vehicles."},
    {"Vehicle profiles",
     "Vehicles contains spawn percentages and the five vehicle-class profiles: bike, motorbike, car, truck and bus. Enter percentages totaling 100 and press Apply mix, or Normalize to 100%. Zero excludes a class. Shares apply to requests; physical insertion can be delayed or rejected.\n\n"
     "Speed and acceleration means and standard deviations, plus Gaussian body length and width, determine new vehicles. Zero standard deviation gives identical values. Units are meters, seconds, m/s and m/s^2.\n\n"
     "Standstill gap, headway, comfortable/emergency deceleration and lateral planning gap are car-following parameters. They are not the old inflated preferred-distance bubble. Editing a profile updates applicable active dynamics; dimensions are sampled at birth. Resample active vehicles updates motion samples.\n\n"
     "Click a vehicle to open its inspector. It shows origin, controller, body size, stopped time and lane-change status. Desired speed and acceleration can be edited for that vehicle. View > Vehicle inspector hides or shows the panel."},
    {"Monitoring and shortcuts",
     "The viewport shows active, completed and open-junction vehicle counts. Expand Live statistics and diagnostics or Monitoring for more detail.\n\n"
     "Stopped time is cumulative simulated time at approximately 0.05 m/s or below. Mean / average, minimum and maximum are per-vehicle totals. Include finished / removed trips controls the sample. Waiting initial SUMO insertions are not road occupancy. Pausing does not add time.\n\n"
     "Native can remove stuck queue leaders after its configured timeout. The separate open-junction timeout deletes stationary vehicles only inside a free junction. Deletion counters distinguish the reasons.\n\n"
     "SUMO collision involvements and teleport starts are separate engine recovery counters. SUMO's 300-second gridlock teleport timeout is enabled. A teleport is not a native deletion, and is not itself a completed trip.\n\n"
     "SHORTCUTS\nF1: open / close this manual.\nEscape: cancel the active map tool; otherwise close the manual.\nWheel: zoom the map.\nMiddle drag: pan.\nLeft-click node: properties.\nCtrl + left-click nodes: select a connected path.\nLeft-click vehicle: inspector.\n\n"
     "Use the bottom status bar for the latest result or error; hover a clipped message to read it fully."}
};
std::string lower(std::string text){for(char& c:text)c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));return text;}
}
void SimulationView::manual(){
    if(!manualOpen_)return;
    ImGui::SetNextWindowSize(ImVec2(850,620),ImGuiCond_FirstUseEver);
    if(ImGui::Begin("User manual",&manualOpen_,ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::SetNextItemWidth(-1);ImGui::InputTextWithHint("##manual_search","Search the manual...",manualSearch_,sizeof(manualSearch_));
        ImGui::Separator();
        ImGui::BeginChild("Topics",ImVec2(230,0),ImGuiChildFlags_Borders);
        for(int i=0;i<static_cast<int>(std::size(chapters));++i)if(ImGui::Selectable(chapters[i].title,manualChapter_==i)){manualChapter_=i;manualSearch_[0]=0;}
        ImGui::EndChild();ImGui::SameLine();ImGui::BeginChild("Manual content",ImVec2(0,0));
        if(manualSearch_[0]) {
            auto query=lower(manualSearch_);bool found=false;
            for(auto& chapter:chapters)if(lower(std::string(chapter.title)+" "+chapter.body).find(query)!=std::string::npos){
                ImGui::SeparatorText(chapter.title);ImGui::TextWrapped("%s",chapter.body);ImGui::Spacing();found=true;
            }
            if(!found)ImGui::TextUnformatted("No matching topics.");
        } else {auto& chapter=chapters[std::clamp(manualChapter_,0,static_cast<int>(std::size(chapters))-1)];ImGui::SeparatorText(chapter.title);ImGui::TextWrapped("%s",chapter.body);}
        ImGui::EndChild();
    }
    ImGui::End();
}
}
