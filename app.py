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
st.title("🚚 Custom Route Optimizer (Plus Codes & Coordinates)")

# Αρχικοποίηση session state
if "optimized_df" not in st.session_state:
    st.session_state.optimized_df = None
if "map_obj" not in st.session_state:
    st.session_state.map_obj = None

# Εισαγωγή (Διευθύνσεις, Συντεταγμένες ή Plus Codes)
default_input = "Ακαδημίας 1, Αθήνα\n8FVC9G8F+6W\n37.9755, 23.7348"
user_input = st.text_area("Σημεία (Διευθύνσεις, Plus Codes ή Συντεταγμένες, μία ανά γραμμή):", value=default_input, height=150)

if st.button("Υπολογισμός Βέλτιστης Διαδρομής"):
    lines = [line.strip() for line in user_input.split("\n") if line.strip()]
    
    if len(lines) < 2:
        st.warning("Παρακαλώ εισάγετε τουλάχιστον 2 σημεία.")
    else:
        geolocator = Nominatim(user_agent="my_route_app_final")
        locations_data = []
        
        with st.spinner("Εντοπισμός σημείων στον χάρτη..."):
            for item in lines:
                try:
                    lat, lng = None, None
                    
                    # 1. Έλεγχος αν είναι Plus Code (περιέχει '+' και δεν είναι απλή διεύθυνση)
                    if "+" in item:
                        code = item.split(",")[0].strip() # Παίρνουμε το κομμάτι πριν το κόμμα αν υπάρχει
                        # Ανεξάρτητα αν είναι full ή short code (τα short codes χρειάζονται reference, οπότε αν είναι full π.χ. 8FVC9G8F+6W)
                        if olc.isValid(code):
                            if olc.isFull(code):
                                decoded = olc.decode(code)
                                lat, lng = decoded.latitudeCenter, decoded.longitudeCenter
                            else:
                                # Αν είναι short code, το συνδυάζουμε με κέντρο την Αθήνα (37.9838, 23.7275)
                                decoded = olc.recover(code, 37.9838, 23.7275)
                                lat, lng = decoded.latitudeCenter, decoded.longitudeCenter
                    
                    # 2. Έλεγχος αν είναι απλές συντεταγμένες (π.χ. "37.98, 23.72")
                    elif "," in item and all(p.strip().replace(".", "", 1).replace("-", "").isdigit() for p in item.split(",")):
                        parts = item.split(",")
                        lat = float(parts[0].strip())
                        lng = float(parts[1].strip())
                        
                    # 3. Αν είναι κανονική διεύθυνση κειμένου
                    else:
                        search_query = item if any(c in item.lower() for c in ["αθήνα", "athens"]) else f"{item}, Αθήνα"
                        location = geolocator.geocode(search_query)
                        if location:
                            lat, lng = location.latitude, location.longitude
                            
                    if lat is not None and lng is not None:
                        locations_data.append({
                            "address": item,
                            "lat": lat,
                            "lng": lng
                        })
                    else:
                        st.error(f"Δεν μπόρεσε να εντοπιστεί το σημείο: {item}")
                        
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

            # Επίλυση με Google OR-Tools (TSP)
            def solve_tsp(matrix):
                manager = pywrapcp.RoutingIndexManager(len(matrix), 1, 0)
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
                    plan.append(manager.IndexToNode(index))
                    return plan
                return None

            with st.spinner("Βελτιστοποίηση διαδρομής με Google OR-Tools..."):
                route_indices = solve_tsp(distance_matrix_int)
                
            if route_indices:
                optimized_df = df.iloc[route_indices[:-1]].reset_index(drop=True)
                optimized_df.index = optimized_df.index + 1
                
                st.session_state.optimized_df = optimized_df
                
                # Δημιουργία Χάρτη
                m = folium.Map(location=[df.iloc[0]["lat"], df.iloc[0]["Lng"] if "Lng" in df.columns else df.iloc[0]["lng"]], zoom_start=13)
                points = []
                for idx, row in optimized_df.iterrows():
                    folium.Marker(
                        [row["lat"], row["lng"]],
                        popup=f"<b>#{idx}: {row['address']}</b>",
                        icon=folium.Icon(color="red" if idx == 1 else "blue", icon="info-sign")
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
            st.markdown(f"**Στάση #{idx}:** {row['address']}")
        with col2:
            map_link = f"https://www.google.com/maps/dir/?api=1&destination={row['lat']},{row['lng']}&travelmode=driving"
            st.markdown(f'<a href="{map_link}" target="_blank"><button style="background-color:#4CAF50; color:white; padding:6px 12px; border:none; border-radius:4px; cursor:pointer; font-size:14px; font-weight:bold;">🚗 Πλοήγηση</button></a>', unsafe_allow_html=True)
        st.divider()

    st.subheader("🗺️ Οπτικοποίηση στον Χάρτη")
    st_folium(st.session_state.map_obj, width=700, height=500, returned_objects=[])