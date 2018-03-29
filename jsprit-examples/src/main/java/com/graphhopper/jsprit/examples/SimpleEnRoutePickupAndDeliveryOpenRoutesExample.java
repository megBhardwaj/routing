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
package com.graphhopper.jsprit.examples;

import com.graphhopper.jsprit.analysis.toolbox.GraphStreamViewer;
import com.graphhopper.jsprit.analysis.toolbox.Plotter;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.box.SchrimpfFactory;
import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.job.Shipment;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleImpl;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleImpl.Builder;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleType;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleTypeImpl;
import com.graphhopper.jsprit.core.reporting.SolutionPrinter;
import com.graphhopper.jsprit.core.util.Coordinate;
import com.graphhopper.jsprit.core.util.Solutions;
import com.graphhopper.jsprit.io.problem.VrpXMLWriter;
import com.graphhopper.jsprit.util.Examples;

import java.util.Collection;


public class SimpleEnRoutePickupAndDeliveryOpenRoutesExample {

    public static void main(String[] args) {
        /*
         * some preparation - create output folder
		 */
        Examples.createOutputFolder();

		/*
         * get a vehicle type-builder and build a type with the typeId "vehicleType" and a capacity of 2
		 */
        VehicleTypeImpl.Builder vehicleTypeBuilder = VehicleTypeImpl.Builder.newInstance("vehicleType").addCapacityDimension(0, 3).setCostPerDistance(10);
        VehicleType vehicleType = vehicleTypeBuilder.build();

		/*
         * get a vehicle-builder and build a vehicle located at (10,10) with type "vehicleType"
		 */
        Builder vehicleBuilder = VehicleImpl.Builder.newInstance("vehicle1");
        vehicleBuilder.setStartLocation(loc(Coordinate.newInstance(5, 9)));
        vehicleBuilder.setType(vehicleType);
        vehicleBuilder.setReturnToDepot(true);
        VehicleImpl vehicle1 = vehicleBuilder.build();

        VehicleTypeImpl.Builder vehicleTypeBuilder1 = VehicleTypeImpl.Builder.newInstance("vehicleType").addCapacityDimension(0, 1).setCostPerDistance(1);
        VehicleType vehicleType1 = vehicleTypeBuilder1.build();

		/*
         * get a vehicle-builder and build a vehicle located at (10,10) with type "vehicleType"
		 */
        Builder vehicleBuilder1 = VehicleImpl.Builder.newInstance("vehicle2");
        vehicleBuilder1.setStartLocation(loc(Coordinate.newInstance(15, 13)));
        vehicleBuilder1.setType(vehicleType1);
        vehicleBuilder1.setReturnToDepot(true);
        VehicleImpl vehicle2 = vehicleBuilder1.build();

        VehicleTypeImpl.Builder vehicleTypeBuilder3 = VehicleTypeImpl.Builder.newInstance("vehicleType").addCapacityDimension(0, 1).setCostPerDistance(1);
        VehicleType vehicleType3 = vehicleTypeBuilder3.build();

		/*
         * get a vehicle-builder and build a vehicle located at (10,10) with type "vehicleType"
		 */
        Builder vehicleBuilder3 = VehicleImpl.Builder.newInstance("vehicle3");
        vehicleBuilder3.setStartLocation(loc(Coordinate.newInstance(5, 7)));
        vehicleBuilder3.setType(vehicleType3);
        vehicleBuilder3.setReturnToDepot(true);
        VehicleImpl vehicle3 = vehicleBuilder3.build();

        VehicleTypeImpl.Builder vehicleTypeBuilder4 = VehicleTypeImpl.Builder.newInstance("vehicleType").addCapacityDimension(0, 1).setCostPerDistance(1);
        VehicleType vehicleType4 = vehicleTypeBuilder4.build();

		/*
         * get a vehicle-builder and build a vehicle located at (10,10) with type "vehicleType"
		 */
        Builder vehicleBuilder4 = VehicleImpl.Builder.newInstance("vehicle4");
        vehicleBuilder4.setStartLocation(loc(Coordinate.newInstance(6, 7)));
        vehicleBuilder4.setType(vehicleType4);
        vehicleBuilder4.setReturnToDepot(true);
        VehicleImpl vehicle4 = vehicleBuilder4.build();


		/*
         * build shipments at the required locations, each with a capacity-demand of 1.
		 * 4 shipments
		 * 1: (5,7)->(6,9)
		 * 2: (5,13)->(6,11)
		 * 3: (15,7)->(14,9)
		 * 4: (15,13)->(14,11)
		 */

        Shipment shipment1 = Shipment.Builder.newInstance("1").addSizeDimension(0, 1).setPickupLocation(loc(Coordinate.newInstance(5, 7))).setDeliveryLocation(loc(Coordinate.newInstance(6, 9))).build();
        Shipment shipment2 = Shipment.Builder.newInstance("2").addSizeDimension(0, 1).setPickupLocation(loc(Coordinate.newInstance(5, 9))).setDeliveryLocation(loc(Coordinate.newInstance(6, 9))).build();

        Shipment shipment3 = Shipment.Builder.newInstance("3").addSizeDimension(0, 1).setPickupLocation(loc(Coordinate.newInstance(6, 7))).setDeliveryLocation(loc(Coordinate.newInstance(6, 9))).build();
        Shipment shipment4 = Shipment.Builder.newInstance("4").addSizeDimension(0, 1).setPickupLocation(loc(Coordinate.newInstance(15, 13))).setDeliveryLocation(loc(Coordinate.newInstance(6, 9))).build();


        VehicleRoutingProblem.Builder vrpBuilder = VehicleRoutingProblem.Builder.newInstance();
        vrpBuilder.addVehicle(vehicle2).addVehicle(vehicle1).addVehicle(vehicle3).addVehicle(vehicle4);
        vrpBuilder.addJob(shipment1).addJob(shipment2).addJob(shipment3).addJob(shipment4);

        VehicleRoutingProblem problem = vrpBuilder.build();

		/*
         * get the algorithm out-of-the-box.
		 */
        VehicleRoutingAlgorithm algorithm = new SchrimpfFactory().createAlgorithm(problem);

		/*
         * and search a solution
		 */
        Collection<VehicleRoutingProblemSolution> solutions = algorithm.searchSolutions();

		/*
         * get the best
		 */
        VehicleRoutingProblemSolution bestSolution = Solutions.bestOf(solutions);

		/*
         * write out problem and solution to xml-file
		 */
        new VrpXMLWriter(problem, solutions).write("output/shipment-problem-with-solution.xml");

		/*
		 * print nRoutes and totalCosts of bestSolution
		 */
        SolutionPrinter.print(problem, bestSolution, SolutionPrinter.Print.VERBOSE);

		/*
		 * plot problem without solution
		 */
        Plotter problemPlotter = new Plotter(problem);
        problemPlotter.plotShipments(true);
        problemPlotter.plot("output/simpleEnRoutePickupAndDeliveryExample_problem.png", "en-route pickup and delivery");

		/*
		 * plot problem with solution
		 */
        Plotter solutionPlotter = new Plotter(problem, Solutions.bestOf(solutions).getRoutes());
        solutionPlotter.plotShipments(true);
        solutionPlotter.plot("output/simpleEnRoutePickupAndDeliveryExample_solution.png", "en-route pickup and delivery");

        new GraphStreamViewer(problem, bestSolution).setRenderShipments(true).setRenderDelay(100).display();
    }

    private static Location loc(Coordinate coordinate) {
        return Location.Builder.newInstance().setCoordinate(coordinate).build();
    }

}
