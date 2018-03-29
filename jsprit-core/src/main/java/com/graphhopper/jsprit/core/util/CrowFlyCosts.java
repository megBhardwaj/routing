/*
 * Licensed to GraphHopper GmbH under one or more contributor
 * license agreements. See the NOTICE file distributed with this work for
 * additional information regarding copyright ownership.
 *
 * GraphHopper GmbH licenses this file to you under the Apache License,
 * Version 2.0 (the "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 *
 */
package com.graphhopper.jsprit.core.util;

import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.cost.AbstractForwardVehicleRoutingTransportCosts;
import com.graphhopper.jsprit.core.problem.cost.TransportDistance;
import com.graphhopper.jsprit.core.problem.driver.Driver;
import com.graphhopper.jsprit.core.problem.vehicle.Vehicle;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import java.util.Collection;
import java.util.Map;
import java.util.HashMap;
/**
 * @author stefan schroeder
 */
public class CrowFlyCosts extends AbstractForwardVehicleRoutingTransportCosts implements TransportDistance {

    public int speed = 1;

    public double detourFactor = 1.0;

    private Locations locations;
    private Collection<Vehicle> vehicles;

    public CrowFlyCosts(Locations locations) {
        super();
        this.locations = locations;

    }

    public CrowFlyCosts(Locations locations, Collection<Vehicle> vehicles) {
        super();
        this.locations = locations;
        this.vehicles = vehicles;
    }

    @Override
    public String toString() {
        return "[name=crowFlyCosts]";
    }

    @Override
    public double getTransportCost(Location from, Location to, double time, Driver driver, Vehicle vehicle) {
        double distance;
        try {
            distance = calculateDistance(from, to);
        } catch (NullPointerException e) {
            throw new NullPointerException("cannot calculate euclidean distance. coordinates are missing. either add coordinates or use another transport-cost-calculator.");
        }

       // double retCost = 1000000;


        double costs = distance;
        if (vehicle != null) {
            if (vehicle.getType() != null) {
                costs = distance * vehicle.getType().getVehicleCostParams().perDistanceUnit ;
                    //vehicle.getType().getVehicleCostParams().fix;
            }
        }
//        try {
//            retCost = costCal(locations.getCoord(from.getId()),locations.getCoord(to.getId()),vehicle);
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//
//        return retCost;

        return 1.2*costs;

    }

    public static double costCal(Coordinate coord1, Coordinate coord2, Vehicle vehicleNew) throws IOException{

//        BufferedReader reader1 = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/Load _file_for_transport_routing.csv"))); //routing_Total_load_file_temp
//        String line1;
//        boolean firstLine1 = true;
//
//        // HashMap<String,Location> vehicleMapE = new HashMap<>();
//
//        while ((line1 = reader1.readLine()) != null) {
//            if (firstLine1) {
//                firstLine1 = false;
//                continue;
//            }
//            String[] tokens1 = line1.split(",");
//            HashMap<Location,> vehicleMapE = new HashMap<>();

    BufferedReader reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/Lane_Master_air_surface.csv")));
    String line;
    boolean firstLine = true;

    double lat1 = coord1.getX();
    double lng1 = coord1.getY();
    double lat2 = coord2.getX();
    double lng2 = coord2.getY();

    double retCost = 1000000000;

        String fleetNew = "22";
            //vehicleNew.getType().getProfile();

    while((line=reader.readLine())!=null)

    {
        if (firstLine) {
            firstLine = false;
            continue;
        }
        String[] tokens = line.split(",");
        if (tokens[4] == null || tokens[5] == null || tokens[6] == null || tokens[7] == null)
            retCost = Double.MAX_VALUE;
        else if (lat1 == Double.parseDouble(tokens[4]) && lng1 == Double.parseDouble(tokens[5]) && lat2 == Double.parseDouble(tokens[6]) && lng2 == Double.parseDouble(tokens[7]) ) {
            if(tokens[0].equals("Surface") && fleetNew.trim().equals(tokens[2].trim())){
                retCost = Double.parseDouble(tokens[3]);
            }

            else
                retCost = Double.parseDouble(tokens[3])*100000;
        }
    }

    reader.close();

        return retCost;
}

    private double calculateDistance(Location fromLocation, Location toLocation) {
        Coordinate from = null;
        Coordinate to = null;
        if (fromLocation.getCoordinate() != null & toLocation.getCoordinate() != null) {
            from = fromLocation.getCoordinate();
            to = toLocation.getCoordinate();
        } else if (locations != null) {
            from = locations.getCoord(fromLocation.getId());
            to = locations.getCoord(toLocation.getId());
        }
        if (from == null || to == null) throw new NullPointerException();
        return calculateDistance(from, to);
    }

    private double calculateDistance(Coordinate from, Coordinate to) {
        return EuclideanDistanceCalculator.calculateDistance(from, to) * detourFactor;
    }

    @Override
    public double getTransportTime(Location from, Location to, double time, Driver driver, Vehicle vehicle) {
        double distance;
        try {
            distance = calculateDistance(from, to);
        } catch (NullPointerException e) {
            throw new NullPointerException("cannot calculate euclidean distance. coordinates are missing. either add coordinates or use another transport-cost-calculator.");
        }
        return distance / vehicle.getType().getMaxVelocity();
    }

    @Override
    public double getDistance(Location from, Location to, double departureTime, Vehicle vehicle) {
        return calculateDistance(from, to);
    }
}
