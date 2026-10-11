import streamlit as st
from geopy.geocoders import Nominatim
import pandas as pd
import requests
import folium
from streamlit_folium import st_folium
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from openlocationcode import openlocationcode as olc

# Τίτλος Εφαρμογής
st.set_page_config(page_title="Route Optimizer", layout="wide")
st.title("🚚 Smart Route Optimizer (Αφετηρία & Στάσεις Παράδοσης)")

# Αρχικοποίηση session state
if "optimized_df" not in st.session_state:
    st.session_state.optimized_df = None
if "map_obj" not in st.session_state:
    st.session_state.map_obj = None

# Εισαγωγή
default_input = "Λεωφόρος Δημοκρατίας 1, Λυκόβρυση (Η Αφετηρία μου)\n8FVC+6W (Στάση 1)\n37.9755, 23.7348 (Στάση 2)"
user_input = st.text_area("1η γραμμή = Αφετηρία | Οι υπόλοιπες = Στάσεις Παράδοσης (Γράψτε οδό, αριθμό και περιοχή π.χ. Λυκόβρυση):", value=default_input, height=150)

if st.button("Υπολογισμός Βέλτιστης Διαδρομής"):
    lines = [line.strip() for line in user_input.split("\n"] if line.strip()] # type: ignore
    # Διόρθωση λίστας γραμμών
    lines = [line.strip() for line in user_input.split("\n") if line.strip()]
    
    if len(lines) < 2:
        st.warning("Παρακαλώ εισάγετε τουλάχιστον την Αφετηρία και μία Στάση.")
    else:
        geolocator = Nominatim(user_agent="my_route_app_flexible")
        locations_data = []
        
        with st.spinner("Εντοπισμός σημείων στον χάρτη..."):
            for item in lines:
                try:
                    lat, lng = None, None
                    # Καθαρισμός κειμένου αν υπάρχει επεξήγηση στην παρένθεση (π.χ. "Λυκόβρυση (Αφετηρία)")
                    clean_item = item.split("(")[0].strip()
                    item_clean_code = clean_item.split(",")[0].strip()
                    
                    # 1. Έλεγχος αν είναι Plus Code (περιέχει '+')
                    if "+" in item_clean_code:
                        if olc.isValid(item_clean_code):
                            code_to_decode = item_clean_code
                            if not olc.isFull(code_to_decode):
                                # Κέντρο αναφοράς Αττικής για σύντομους κωδικούς (π.χ. Λυκόβρυση/Αθήνα)
                                code_to_decode = olc.recoverNearest(code_to_decode, 38.05, 23.77)
                            
                            if olc.isFull(code_to_decode):
                                decoded = olc.decode(code_to_decode)
                                lat, lng = decoded.latitudeCenter, decoded.longitudeCenter
                                
                    # 2. Έλεγχος αν είναι απλές συντεταγμένες (π.χ. "38.06, 23.78")
                    elif "," in clean_item and all(p.strip().replace(".", "", 1).replace("-", "").isdigit() for p in clean_item.split(",")):
                        parts = clean_item.split(",")
                        lat = float(parts[0].strip())
                        lng = float(parts[1].strip())
                        
                    # 3. Αν είναι κανονική διεύθυνση κειμένου (Ψάχνει ακριβώς αυτό που δίνετε, χωρίς να προσθέτει Αθήνα αναγκαστικά)
                    else:
                        location = geolocator.geocode(clean_item)
                        if location:
                            lat, lng = location.latitude, location.longitude
                            
                    if lat is not None and lng is not None:
                        locations_data.append({
                            "address": item,
                            "lat": lat,
                            "lng": lng
                        })
                    else:
                        st.error(f"Δεν μπόρεσε να εντοπιστεί το σημείο: {item}. Δοκιμάστε να προσθέσετε πόλη/περιοχή (π.χ. Λυκόβρυση).")
                        
                except Exception as e:
                    st.error(f"Σφάλμα για το σημείο '{item}': {e}")
                    
        if len(locations_data) == len(lines):
            df = pd.DataFrame(locations_data)
            
            # Δημιουργία πίνακα αποστάσεων μέσω OSRM API
            with st.spinner("Υπολογισμός αποστάσεων δρόμου..."):
                coords = ";".join([f"{row['lng']},{row['lat']}" for index, row in df.iterrows()])
                url = f"http://router.project-osrm.org/table/v1/driving/{coords}?annotations=distance"
                
                response = requests.get(url)
                if response.status_code == 200:
                    data = response.json()
                    distance_matrix = data['distances']
                    distance_matrix_int = [[int(val) for val in row] for row in distance_matrix]
                else:
                    st.error("Αποτυχία επικοινωνίας με τον server χαρτών.")
                    st.stop()

            # Επίλυση με Google OR-Tools κρατώντας σταθερή την Αφετηρία (Index 0)
            def solve_tsp_with_fixed_start(matrix):
                num_nodes = len(matrix)
                manager = pywrapcp.RoutingIndexManager(num_nodes, 1, 0) # 0 είναι η αφετηρία μας
                routing = pywrapcp.RoutingModel(manager)

                def distance_callback(from_index, to_index):
                    from_node = manager.IndexToNode(from_index)
                    to_node = manager.IndexToNode(to_index)
                    return matrix[from_node][to_node]

                transit_callback_index = routing.RegisterTransitCallback(distance_callback)
                routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

                search_parameters = pywrapcp.DefaultRoutingSearchParameters()
                search_parameters.first_solution_strategy = (
                    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

                solution = routing.SolveWithParameters(search_parameters)
                
                if solution:
                    index = routing.Start(0)
                    plan = []
                    while not routing.IsEnd(index):
                        plan.append(manager.IndexToNode(index))
                        index = solution.Value(routing.NextVar(index))
                    return plan[:-1] if len(plan) == num_nodes + 1 else plan
                return None

            with st.spinner("Βελτιστοποίηση διαδρομής παραδόσεων..."):
                route_indices = solve_tsp_with_fixed_start(distance_matrix_int)
                
            if route_indices:
                full_route = route_indices.copy()
                for i in range(len(df)):
                    if i not in full_route:
                        full_route.append(i)
                
                optimized_df = df.iloc[full_route].reset_index(drop=True)
                st.session_state.optimized_df = optimized_df
                
                # Δημιουργία Χάρτη με κέντρο την αφετηρία
                m = folium.Map(location=[df.iloc[0]["lat"], df.iloc[0]["lng"]], zoom_start=13)
                points = []
                for idx, row in optimized_df.iterrows():
                    marker_color = "red" if idx == 0 else "blue"
                    popup_text = f"<b>ΑΦΕΤΗΡΙΑ: {row['address']}</b>" if idx == 0 else f"<b>Στάση #{idx}: {row['address']}</b>"
                    
                    folium.Marker(
                        [row["lat"], row["lng"]],
                        popup=popup_text,
                        icon=folium.Icon(color=marker_color, icon="info-sign" if idx > 0 else "home")
                    ).add_to(m)
                    points.append([row["lat"], row["lng"]])
                
                folium.PolyLine(points, color="purple", weight=4, opacity=0.8).add_to(m)
                st.session_state.map_obj = m
            else:
                st.error("Δεν βρέθηκε λύση από τον αλγόριθμο.")

# Εμφάνιση αποτελεσμάτων από το session_state
if st.session_state.optimized_df is not None:
    st.success("Η διαδρομή βελτιστοποιήθηκε επιτυχώς!")
    st.subheader("📍 Σειρά επίσκεψης σημείων & Πλοήγηση:")
    
    for idx, row in st.session_state.optimized_df.iterrows():
        col1, col2 = st.columns([3, 1])
        with col1:
            if idx == 0:
                st.markdown(f"🏠 **ΑΦΕΤΗΡΙΑ (Τοποθεσία σας):** {row['address']}")
            else:
                st.markdown(f"📦 **Στάση #{idx}:** {row['address']}")
        with col2:
            map_link = f"https://www.google.com/maps/dir/?api=1&destination={row['lat']},{row['lng']}&travelmode=driving"
            st.markdown(f'<a href="{map_link}" target="_blank"><button style="background-color:#4CAF50; color:white; padding:6px 12px; border:none; border-radius:4px; cursor:pointer; font-size:14px; font-weight:bold;">🚗 Πλοήγηση</button></a>', unsafe_allow_html=True)
        st.divider()

    st.subheader("🗺️ Οπτικοποίηση στον Χάρτη")
    st_folium(st.session_state.map_obj, width=700, height=500, returned_objects=[])