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

import java.io.*;


/**
 * @author stefan schroeder
 */
public class EuclideanCosts extends AbstractForwardVehicleRoutingTransportCosts implements TransportDistance {

    public int speed = 1;

    public double detourFactor = 1.0;

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
        double costs = distance;
        if (vehicle != null) {
            if (vehicle.getType() != null) {
                /*
                if (vehicle.getId().contains("AIR")) {
                    Integer load_factor=0;
                    Double R=0.0;

                    BufferedReader reader = null; //routing_Total_load_file_temp
                    try {
                        reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/air_fleet_master.csv")));
                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                    }
                    String line;
                    boolean firstLine = true;
                    try {
                        while ((line = reader.readLine()) != null) {
                            if (firstLine) {
                                firstLine = false;
                                continue;
                            }
                            String[] tokens = line.split(",");

                            if(from.getCoordinate().getX()==Double.parseDouble( tokens[1]) && from.getCoordinate().getY()==Double.parseDouble( tokens[2])) {

                                if (to.getCoordinate().getX() == Double.parseDouble(tokens[3]) && to.getCoordinate().getY() == Double.parseDouble(tokens[4])) {


                                    //
                                    R=Double.parseDouble(tokens[6]);

                                    BufferedReader reader_load = null; //routing_Total_load_file_temp
                                    try {
                                        reader_load = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/national_72_lanes_input_with_split_on_new_capacity_v2.csv")));
                                    } catch (FileNotFoundException e) {
                                        e.printStackTrace();
                                    }
                                    String line_load;
                                    boolean firstLine_load = true;
                                    try {
                                        while ((line_load = reader_load.readLine()) != null) {
                                            if (firstLine_load) {
                                                firstLine_load = false;
                                                continue;
                                            }
                                            String[] tokens_load = line_load.split(",");

                                            if (from.getCoordinate().getX() == Double.parseDouble(tokens_load[2]) && from.getCoordinate().getY() == Double.parseDouble(tokens_load[3]) && to.getCoordinate().getX() == Double.parseDouble(tokens_load[4]) && to.getCoordinate().getY() == Double.parseDouble(tokens_load[5]) && Double.parseDouble(tokens[5])>Double.parseDouble(tokens_load[8])) {

                                                load_factor=Integer.parseInt( tokens_load[9]);
                                            }
                                        }
                                    } catch (IOException e) {
                                        e.printStackTrace();
                                    }

                                    //
                                }
                            }
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }


                    //get load


                    costs=load_factor*R*1.15;

                }
                    else
                */
                    costs = distance * vehicle.getType().getVehicleCostParams().perDistanceUnit;
            }
        }
        return costs;
    }

    private double calculateDistance(Location fromLocation, Location toLocation) {
        Coordinate from = null;
        Coordinate to = null;
        if (fromLocation.getCoordinate() != null & toLocation.getCoordinate() != null) {
            from = fromLocation.getCoordinate();
            to = toLocation.getCoordinate();
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
        return distance / speed;
    }

    @Override
    public double getDistance(Location from, Location to, double departureTime, Vehicle vehicle) {
        return calculateDistance(from, to);
    }
}
