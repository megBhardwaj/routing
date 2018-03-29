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
package com.graphhopper.jsprit.core.util;


import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleImpl;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleType;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleTypeImpl;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

import static java.lang.Math.atan2;
import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.sin;
import static java.lang.StrictMath.sqrt;

public class EuclideanDistanceCalculator {

    public static double calculateDistance(Coordinate coord1, Coordinate coord2) {
        //double xDiff = coord1.getX() - coord2.getX();
        //double yDiff = coord1.getY() - coord2.getY();


        double lat1 = coord1.getX();
        double lng1 = coord1.getY();
        double lat2 = coord2.getX();
        double lng2 = coord2.getY();
/*
        double dis=0.0;

        try {
            dis=calcDis(lat1,lng1,lat2,lng2);
        } catch (IOException e) {
            e.printStackTrace();
        }

        return dis;

    }



    public static double calcDis(double lat1, double lng1, double lat2, double lng2) throws IOException{

        double dis = 0.0;
        BufferedReader reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/lat_lng_google_dis_blr_zonal.csv"))); //routing_Total_load_file_temp//load_input_with_reduced_hops_for_routing
        String line;
        boolean firstLine = true;

        while ((line = reader.readLine()) != null) {
            if (firstLine) {
                firstLine = false;
                continue;
            }
            String[] tokens = line.split(",");

            if (Double.parseDouble(tokens[0]) == lat1) {
                if (Double.parseDouble(tokens[1]) == lng1) {
                    if (Double.parseDouble(tokens[2]) == lat2) {
                        if (Double.parseDouble(tokens[3]) == lng2) {
                            dis = Double.parseDouble(tokens[4]);
                        }
                    }
                }
            }

           // reader.close();


        }

        reader.close();
        return dis;


    }



        */
        double earthRadius = 3958.75;
        double dLat = ToRadians(lat2 - lat1);
        double dLng = ToRadians(lng2 - lng1);
        double a = sin(dLat / 2) * sin(dLat / 2) +
            cos(ToRadians(lat1)) * cos(ToRadians(lat2)) *
                sin(dLng / 2) * sin(dLng / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        double dist = earthRadius * c;
        double kmeterConversion = 1.609;
        //double retCost; //= Double.MAX_VALUE;
        //dist * kmeterConversion;
        //double retCost;

        return dist * kmeterConversion * 1000 * 1.3;
        // return Math.sqrt((xDiff * xDiff) + (yDiff * yDiff));

//        try {
//            retCost = costCal(coord1,coord2);
//        } catch (IOException e) {
//            retCost=0;
//        }
//        return retCost;
    }


    static double ToRadians(double degrees) {
        double radians = degrees * 3.1415926535897932385 / 180;
        return radians;
    }


}

//    public static double costCal(Coordinate coord1, Coordinate coord2) throws IOException{
//        BufferedReader reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/Lane_master.csv")));
//        String line;
//        boolean firstLine = true;
//
//        double lat1 = coord1.getX();
//        double lng1 = coord1.getY();
//        double lat2 = coord2.getX();
//        double lng2 = coord2.getY();
//
//        double retCost=1000000000;
//
//        while ((line = reader.readLine()) != null) {
//            if (firstLine) {
//                firstLine = false;
//                continue;
//            }
//            String[] tokens = line.split(",");
//            if(tokens[3]==null || tokens[4]==null || tokens[5]==null || tokens[6]==null)
//                retCost=0;
//           else if (lat1 == Double.parseDouble(tokens[3]) && lng1 == Double.parseDouble(tokens[4]) && lat2 == Double.parseDouble(tokens[5]) && lng2 == Double.parseDouble(tokens[6])) {
//                retCost = Double.parseDouble(tokens[2]);
//            }
//        }
//        reader.close();
//
//        return retCost;
//
//    }








//    double DirectDistance(double lat1, double lng1, double lat2, double lng2)
//    {
//
//    }

//}

// #define LOCAL_PI 3.1415926535897932385

