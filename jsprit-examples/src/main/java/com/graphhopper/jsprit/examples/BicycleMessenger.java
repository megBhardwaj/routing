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


import com.graphhopper.jsprit.analysis.toolbox.AlgorithmSearchProgressChartListener;
import com.graphhopper.jsprit.analysis.toolbox.GraphStreamViewer;
import com.graphhopper.jsprit.analysis.toolbox.GraphStreamViewer.Label;
import com.graphhopper.jsprit.analysis.toolbox.Plotter;
import com.graphhopper.jsprit.core.algorithm.VehicleRoutingAlgorithm;
import com.graphhopper.jsprit.core.algorithm.box.Jsprit;
import com.graphhopper.jsprit.core.algorithm.recreate.IncreasingAbsoluteFixedCosts;
import com.graphhopper.jsprit.core.algorithm.state.InternalStates;
import com.graphhopper.jsprit.core.algorithm.state.StateId;
import com.graphhopper.jsprit.core.algorithm.state.StateManager;
import com.graphhopper.jsprit.core.algorithm.state.StateUpdater;
import com.graphhopper.jsprit.core.problem.Location;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem.Builder;
import com.graphhopper.jsprit.core.problem.VehicleRoutingProblem.FleetSize;
import com.graphhopper.jsprit.core.problem.constraint.ConstraintManager;
import com.graphhopper.jsprit.core.problem.constraint.HardActivityConstraint;
import com.graphhopper.jsprit.core.problem.constraint.HardRouteConstraint;
import com.graphhopper.jsprit.core.problem.cost.VehicleRoutingTransportCosts;
import com.graphhopper.jsprit.core.problem.driver.Driver;
import com.graphhopper.jsprit.core.problem.driver.DriverImpl;
import com.graphhopper.jsprit.core.problem.job.Job;
import com.graphhopper.jsprit.core.problem.job.Shipment;
import com.graphhopper.jsprit.core.problem.misc.JobInsertionContext;
import com.graphhopper.jsprit.core.problem.solution.VehicleRoutingProblemSolution;
import com.graphhopper.jsprit.core.problem.solution.route.VehicleRoute;
import com.graphhopper.jsprit.core.problem.solution.route.activity.ReverseActivityVisitor;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TimeWindow;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TourActivity;
import com.graphhopper.jsprit.core.problem.solution.route.activity.TourActivity.JobActivity;
import com.graphhopper.jsprit.core.problem.solution.route.state.RouteAndActivityStateGetter;
import com.graphhopper.jsprit.core.problem.vehicle.Vehicle;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleImpl;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleType;
import com.graphhopper.jsprit.core.problem.vehicle.VehicleTypeImpl;
import com.graphhopper.jsprit.core.reporting.SolutionPrinter;
import com.graphhopper.jsprit.core.util.Coordinate;
import com.graphhopper.jsprit.core.util.CrowFlyCosts;
import com.graphhopper.jsprit.core.util.Solutions;
import com.graphhopper.jsprit.util.Examples;
import jdk.nashorn.internal.objects.annotations.Getter;
import jdk.nashorn.internal.objects.annotations.Setter;
import scala.Int;
import scala.util.parsing.combinator.testing.Str;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.StringTokenizer;


/**
 * This class provides the/a solution to the following problem:
 * <p>
 * Statement of the problem (see Stackoverflow: http://stackoverflow.com/questions/19080537/bicycle-messenger-tsppd-with-optaplanner/20412598#20412598):
 * <p>
 * Optimize the routes for a bicycle messenger service!
 * Assume 5 messengers that have to pick up 30 envelopes distributed through the city. These 5 messengers are distributed through the city as well. Thus
 * there is no single depot and they do not need to go back to their original starting location.
 * <p>
 * Additional hard constraints:
 * 1) Every messenger can carry up to fifteen envelopes
 * 2) The way an evelopes travels should be less than three times the direct route (so delivery does not take too long)
 * <p>
 * Thus this problem is basically a Capacitated VRP with Pickups and Deliveries, Multiple Depots, Open Routes and Time Windows/Restrictions.
 *
 * @author stefan schroeder
 */
public class BicycleMessenger {

    /**
     * Hard constraint: delivery of envelope must not take longer than 3*bestDirect (i.e. fastest messenger on direct delivery)
     *
     * @author stefan
     */
    static class ThreeTimesLessThanBestDirectRouteConstraint implements HardActivityConstraint {

        private final VehicleRoutingTransportCosts routingCosts;

        private final RouteAndActivityStateGetter stateManager;

        //jobId map direct-distance by nearestMessenger
        private final Map<String, Double> bestMessengers;

        private final StateId latest_act_arrival_time_stateId;

        public ThreeTimesLessThanBestDirectRouteConstraint(StateId latest_act_arrival_time, Map<String, Double> nearestMessengers, VehicleRoutingTransportCosts routingCosts, RouteAndActivityStateGetter stateManager) {
            this.bestMessengers = nearestMessengers;
            this.routingCosts = routingCosts;
            this.stateManager = stateManager;
            this.latest_act_arrival_time_stateId = latest_act_arrival_time;
        }

        @Override
        public ConstraintsStatus fulfilled(JobInsertionContext iFacts, TourActivity prevAct, TourActivity newAct, TourActivity nextAct, double prevActDepTime) {
            //make sure vehicle can manage direct path
            double arrTime_at_nextAct_onDirectRoute = prevActDepTime + routingCosts.getTransportTime(prevAct.getLocation(), nextAct.getLocation(), prevActDepTime, iFacts.getNewDriver(), iFacts.getNewVehicle());
            Double latest_arrTime_at_nextAct = stateManager.getActivityState(nextAct, latest_act_arrival_time_stateId, Double.class);
            if (latest_arrTime_at_nextAct == null)
                latest_arrTime_at_nextAct = nextAct.getTheoreticalLatestOperationStartTime();
            if (arrTime_at_nextAct_onDirectRoute > latest_arrTime_at_nextAct) {
                //constraint can never be fulfilled anymore, thus .NOT_FULFILLED_BREAK
                return ConstraintsStatus.NOT_FULFILLED_BREAK;
            }

            double arrTime_at_newAct = prevActDepTime + routingCosts.getTransportTime(prevAct.getLocation(), newAct.getLocation(), prevActDepTime, iFacts.getNewDriver(), iFacts.getNewVehicle());
            //local impact
            //no matter whether it is a pickupShipment or deliverShipment activities. both arrivalTimes must be < 3*best.
            double directTimeOfNearestMessenger = bestMessengers.get(((JobActivity) newAct).getJob().getId());
            if (arrTime_at_newAct > 10000000 * directTimeOfNearestMessenger) {
                //not fulfilled AND it can never be fulfilled anymore by going forward in route, thus NOT_FULFILLED_BREAK
                return ConstraintsStatus.NOT_FULFILLED_BREAK;
            }

            //impact on whole route, since insertion of newAct shifts all subsequent activities forward in time
            double departureTime_at_newAct = arrTime_at_newAct + newAct.getOperationTime();
            double latest_arrTime_at_newAct = latest_arrTime_at_nextAct - routingCosts.getTransportTime(newAct.getLocation(), nextAct.getLocation(), departureTime_at_newAct, iFacts.getNewDriver(), iFacts.getNewVehicle());
            if (arrTime_at_newAct > latest_arrTime_at_newAct) {
                return ConstraintsStatus.NOT_FULFILLED;
            }

            double arrTime_at_nextAct = departureTime_at_newAct + routingCosts.getTransportTime(newAct.getLocation(), nextAct.getLocation(), departureTime_at_newAct, iFacts.getNewDriver(), iFacts.getNewVehicle());
            //here you need an activity state
            if (arrTime_at_nextAct > latest_arrTime_at_nextAct) {
                return ConstraintsStatus.NOT_FULFILLED;
            }
            return ConstraintsStatus.FULFILLED;
        }

    }

    /**
     * When inserting the activities of an envelope which are pickup and deliver envelope, this constraint makes insertion procedure to ignore messengers that are too far away to meet the 3*directTime-Constraint.
     * <p>
     * <p>one does not need this constraint. but it is faster. the earlier the solution-space can be constraint the better/faster.
     *
     * @author schroeder
     */
    static class IgnoreMessengerThatCanNeverMeetTimeRequirements implements HardRouteConstraint {

        private final Map<String, Double> bestMessengers;

        private final VehicleRoutingTransportCosts routingCosts;

        public IgnoreMessengerThatCanNeverMeetTimeRequirements(Map<String, Double> bestMessengers, VehicleRoutingTransportCosts routingCosts) {
            super();
            this.bestMessengers = bestMessengers;
            this.routingCosts = routingCosts;
        }

        @Override
        public boolean fulfilled(JobInsertionContext insertionContext) {
            double timeOfDirectRoute = getTimeOfDirectRoute(insertionContext.getJob(), insertionContext.getNewVehicle(), routingCosts);
            double timeOfNearestMessenger = bestMessengers.get(insertionContext.getJob().getId());
            return !(timeOfDirectRoute > 50 * timeOfNearestMessenger);
        }

    }

    /**
     * updates the state "latest-activity-start-time" (required above) once route/activity states changed, i.e. when removing or inserting an envelope-activity
     * <p>
     * <p>thus once either the insertion-procedure starts or an envelope has been inserted, this visitor runs through the route in reverse order (i.e. starting with the end of the route) and
     * calculates the latest-activity-start-time (or latest-activity-arrival-time) which is the time to just meet the constraints of subsequent activities.
     *
     * @author schroeder
     */
    static class UpdateLatestActivityStartTimes implements StateUpdater, ReverseActivityVisitor {

        private final StateManager stateManager;

        private final VehicleRoutingTransportCosts routingCosts;

        private final Map<String, Double> bestMessengers;

        private VehicleRoute route;

        private TourActivity prevAct;

        private double latest_arrTime_at_prevAct;

        private final StateId latest_act_arrival_time_stateId;

        public UpdateLatestActivityStartTimes(StateId latest_act_arrival_time, StateManager stateManager, VehicleRoutingTransportCosts routingCosts, Map<String, Double> bestMessengers) {
            super();
            this.stateManager = stateManager;
            this.routingCosts = routingCosts;
            this.bestMessengers = bestMessengers;
            this.latest_act_arrival_time_stateId = latest_act_arrival_time;
        }

        @Override
        public void begin(VehicleRoute route) {
            this.route = route;
            latest_arrTime_at_prevAct = route.getEnd().getTheoreticalLatestOperationStartTime();
            prevAct = route.getEnd();
        }

        @Override
        public void visit(TourActivity currAct) {
            double timeOfNearestMessenger = bestMessengers.get(((JobActivity) currAct).getJob().getId());
            double potential_latest_arrTime_at_currAct =
                latest_arrTime_at_prevAct - routingCosts.getBackwardTransportTime(currAct.getLocation(), prevAct.getLocation(), latest_arrTime_at_prevAct, route.getDriver(), route.getVehicle()) - currAct.getOperationTime();
            double latest_arrTime_at_currAct = Math.min(10000000 * timeOfNearestMessenger, potential_latest_arrTime_at_currAct);
            stateManager.putActivityState(currAct, latest_act_arrival_time_stateId, latest_arrTime_at_currAct);
            assert currAct.getArrTime() <= latest_arrTime_at_currAct : "this must not be since it breaks condition; actArrTime: " + currAct.getArrTime() + " latestArrTime: " + latest_arrTime_at_currAct + " vehicle: " + route.getVehicle().getId();
            latest_arrTime_at_prevAct = latest_arrTime_at_currAct;
            prevAct = currAct;
        }

        @Override
        public void finish() {
        }

    }

    /**
     * @throws IOException
     */
    public static void main(String[] args) throws IOException {
        Examples.createOutputFolder();

		/*
        build the problem
		 */
        VehicleRoutingProblem.Builder problemBuilder = VehicleRoutingProblem.Builder.newInstance();
        problemBuilder.setFleetSize(FleetSize.FINITE);
        readEnvelopes(problemBuilder);
        readMessengers(problemBuilder);
        //readDrivers(problemBuilder);
        //add constraints to problem
        VehicleRoutingTransportCosts routingCosts = new CrowFlyCosts(problemBuilder.getLocations()); //which is the default VehicleRoutingTransportCosts in builder above

       // VehicleRoutingTransportCosts routeCost = ;
      //  routeCost.getTransportCost(problemBuilder.getLocations(),problemBuilder.getLocations(),0.0,null,problemBuilder.getAddedVehicles());
        problemBuilder.setRoutingCost(routingCosts);
        //finally build the problem
//        problemBuilder.addPenaltyVehicles(20.0,50000);
        VehicleRoutingProblem bicycleMessengerProblem = problemBuilder.build();

        /*
        define states and constraints
         */
        //map mapping nearest messengers, i.e. for each envelope the direct-delivery-time with the fastest messenger is stored here
        Map<String, Double> nearestMessengers = getNearestMessengers(routingCosts, problemBuilder.getAddedJobs(), problemBuilder.getAddedVehicles());

        //define stateManager to update the required activity-state: "latest-activity-start-time"
        StateManager stateManager = new StateManager(bicycleMessengerProblem);
        //create state
        StateId latest_act_arrival_time_stateId = stateManager.createStateId("latest-act-arrival-time");
        //and make sure you update the activity-state "latest-activity-start-time" the way it is defined above
        stateManager.addStateUpdater(new UpdateLatestActivityStartTimes(latest_act_arrival_time_stateId, stateManager, routingCosts, nearestMessengers));
        stateManager.updateLoadStates();

        ConstraintManager constraintManager = new ConstraintManager(bicycleMessengerProblem, stateManager);
        constraintManager.addLoadConstraint();
        //constraintManager.addConstraint(new ThreeTimesLessThanBestDirectRouteConstraint(latest_act_arrival_time_stateId, nearestMessengers, routingCosts, stateManager), ConstraintManager.Priority.CRITICAL);
        constraintManager.addConstraint(new IgnoreMessengerThatCanNeverMeetTimeRequirements(nearestMessengers, routingCosts));

        VehicleRoutingAlgorithm algorithm = Jsprit.Builder.newInstance(bicycleMessengerProblem)
            .setStateAndConstraintManager(stateManager,constraintManager).buildAlgorithm();

        algorithm.setMaxIterations(100);

//		VehicleRoutingAlgorithm algorithm = Jsprit.Builder.newInstance(bicycleMessengerProblem)
//				.setStateAndConstraintManager(stateManager, constraintManager)
//				.setProperty(Jsprit.Parameter.THREADS.toString(), "6")
////				.setProperty(Jsprit.Strategy.RADIAL_BEST.toString(), "0.25")
////				.setProperty(Jsprit.Strategy.WORST_BEST.toString(), "0.25")
////				.setProperty(Jsprit.Strategy.CLUSTER_BEST.toString(), "0.25")
////				.setProperty(Jsprit.Strategy.RANDOM_BEST.toString(), "0.")
////				.setProperty(Jsprit.Strategy.RANDOM_REGRET.toString(), "1.")
//				.setProperty(Jsprit.Parameter.INSERTION_NOISE_LEVEL.toString(),"0.01")
//				.setProperty(Jsprit.Parameter.INSERTION_NOISE_PROB.toString(), "0.2")
////				.setProperty(Jsprit.Parameter.THRESHOLD_ALPHA.toString(),"0.1")
//				.buildAlgorithm();
//		algorithm.setMaxIterations(5000);

//        VariationCoefficientTermination prematureAlgorithmTermination = new VariationCoefficientTermination(200, 0.001);
//        algorithm.setPrematureAlgorithmTermination(prematureAlgorithmTermination);
//        algorithm.addListener(prematureAlgorithmTermination);
        algorithm.addListener(new AlgorithmSearchProgressChartListener("output/progress.png"));


        //search
        Collection<VehicleRoutingProblemSolution> solutions = algorithm.searchSolutions();

        //this is just to ensure that solution meet the above constraints
  //      validateSolution(Solutions.bestOf(solutions), bicycleMessengerProblem, nearestMessengers);

        SolutionPrinter.print(bicycleMessengerProblem, Solutions.bestOf(solutions), SolutionPrinter.Print.VERBOSE);

        //you may want to plot the problem
        Plotter plotter = new Plotter(bicycleMessengerProblem);
//		plotter.setBoundingBox(10000, 47500, 20000, 67500);
        plotter.plotShipments(true);
        plotter.plot("output/bicycleMessengerProblem.png", "bicycleMessenger");

        //and the problem as well as the solution
        Plotter plotter1 = new Plotter(bicycleMessengerProblem, Solutions.bestOf(solutions));
        plotter1.setLabel(Plotter.Label.ID);
        plotter1.plotShipments(false);
//		plotter1.setBoundingBox(5000, 45500, 25000, 66500);
        plotter1.plot("output/bicycleMessengerSolution.png", "bicycleMessenger");

        //and write out your solution in xml
//		new VrpXMLWriter(bicycleMessengerProblem, solutions).write("output/bicycleMessenger.xml");


        new GraphStreamViewer(bicycleMessengerProblem).labelWith(Label.ID).setRenderShipments(true).setRenderDelay(150).display();
//
        new GraphStreamViewer(bicycleMessengerProblem, Solutions.bestOf(solutions)).setGraphStreamFrameScalingFactor(1.5).setCameraView(12500, 55000, 0.25).labelWith(Label.ACTIVITY).setRenderShipments(true).setRenderDelay(150).display();

    }

    //if you wanne run this enable assertion by putting an '-ea' in your vmargument list - Run As --> Run Configurations --> (x)=Arguments --> VM arguments: -ea
    private static void validateSolution(VehicleRoutingProblemSolution bestOf, VehicleRoutingProblem bicycleMessengerProblem, Map<String, Double> nearestMessengers) {
        for (VehicleRoute route : bestOf.getRoutes()) {
            for (TourActivity act : route.getActivities()) {
                if (act.getArrTime() > 100 * nearestMessengers.get(((JobActivity) act).getJob().getId())) {
                    SolutionPrinter.print(bicycleMessengerProblem, bestOf, SolutionPrinter.Print.VERBOSE);
                    throw new IllegalStateException("three times less than ... constraint broken. this must not be. act.getArrTime(): " + act.getArrTime() + " allowed: " + 6 * nearestMessengers.get(((JobActivity) act).getJob().getId()));
                }
            }
        }
    }

    static Map<String, Double> getNearestMessengers(VehicleRoutingTransportCosts routingCosts, Collection<Job> envelopes, Collection<Vehicle> messengers) {
        Map<String, Double> nearestMessengers = new HashMap<String, Double>();
        for (Job envelope : envelopes) {
            double minDirect = Double.MAX_VALUE;
            for (Vehicle m : messengers) {
                double direct = getTimeOfDirectRoute(envelope, m, routingCosts);
                if (direct < minDirect) {
                    minDirect = direct;
                }
            }
            nearestMessengers.put(envelope.getId(), minDirect);
        }
        return nearestMessengers;
    }

    static double getTimeOfDirectRoute(Job job, Vehicle v, VehicleRoutingTransportCosts routingCosts) {
        Shipment envelope = (Shipment) job;
        return routingCosts.getTransportTime(v.getStartLocation(), envelope.getPickupLocation(), 0.0, DriverImpl.noDriver(), v) +
            routingCosts.getTransportTime(envelope.getPickupLocation(), envelope.getDeliveryLocation(), 0.0, DriverImpl.noDriver(), v);
    }

    static HashMap<String,Location> vehicleMap = new HashMap<>();
    //Location vehicleStartLocation = Location.newInstance(0,0);

    private static void readEnvelopes(Builder problemBuilder) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/nat_s3.csv"))); //AMD_Zonal_input_agrr_t1//routing_Total_load_file_temp//load_input_with_reduced_hops_for_routing//load_input_with_reduced_hops_for_routing_load_split//national_72_lanes_input_with_split_on_new_capacity_v2//NAT_input_v1//nat_march_s1
        String line;
        boolean firstLine = true;

       // HashMap<String,Location> vehicleMapE = new HashMap<>();

        while ((line = reader.readLine()) != null) {
            if (firstLine) {
                firstLine = false;
                continue;
            }
            String[] tokens = line.split(",");
            TimeWindow tw = new TimeWindow(0,432000); //Double.parseDouble(tokens[10])
            //define your envelope which is basically a shipment from A to B
            Shipment envelope = Shipment.Builder.newInstance(tokens[1]).addSizeDimension(0,Integer.parseInt(tokens[9]))
                //.setDeliveryTimeWindow(tw)
                //.setDgFlag(Boolean.parseBoolean(tokens[1]))
                .addPickupTimeWindow(Double.parseDouble(tokens[8]),Double.parseDouble(tokens[10]))//Double.parseDouble(tokens[8]),172800-Double.parseDouble( tokens[8])//Double.parseDouble(tokens[8])+54000
                .addDeliveryTimeWindow(Double.parseDouble(tokens[8]),Double.parseDouble(tokens[10]))//201600//Double.parseDouble(tokens[8])+54000
                .setPriority(1)
                .setDeliveryServiceTime(2700)//900+((Double.parseDouble(tokens[9])/(20*8))*60))
                .setPickupServiceTime(300+((Double.parseDouble(tokens[9])/(20*8))*60))
                .setPickupLocation(Location.Builder.newInstance().setCoordinate(Coordinate.newInstance(Double.parseDouble(tokens[2]), Double.parseDouble(tokens[3]))).build())
                .setDeliveryLocation(Location.Builder.newInstance().setCoordinate(Coordinate.newInstance(Double.parseDouble(tokens[4]), Double.parseDouble(tokens[5]))).build()).build();
            problemBuilder.addJob(envelope);


            if(vehicleMap.get(tokens[6])==null)
            vehicleMap.put(tokens[6],Location.Builder.newInstance().setCoordinate(Coordinate.newInstance(Double.parseDouble(tokens[2]), Double.parseDouble(tokens[3]))).build());

        }
        reader.close();


    }

    private static void  readDrivers(Builder problemBuilder) throws IOException {

        BufferedReader reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/drivers.csv"))); //AMD_Zonal_input_agrr_t1//routing_Total_load_file_temp//load_input_with_reduced_hops_for_routing//load_input_with_reduced_hops_for_routing_load_split//national_72_lanes_input_with_split_on_new_capacity_v2
        String line;
        boolean firstLine = true;

        // HashMap<String,Location> vehicleMapE = new HashMap<>();

        while ((line = reader.readLine()) != null) {
            if (firstLine) {
                firstLine = false;
                continue;
            }
            String[] tokens = line.split(",");


            Driver driver = DriverImpl.Builder.newInstance(tokens[0]).setStartLocation(Location.Builder.newInstance().setCoordinate(Coordinate.newInstance(Double.parseDouble(tokens[2]), Double.parseDouble(tokens[3]))).build())
                            .setEarliestStart(Double.parseDouble(tokens[4])).build();


            problemBuilder.addDriver(driver);
        }
    }

    private static void readMessengers(Builder problemBuilder) throws IOException {
//        BufferedReader reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/fleet_master_file.csv")));
//        String line;
//        boolean firstLine = true;
       // VehicleType messengerType = VehicleTypeImpl.Builder.newInstance("messengerType").addCapacityDimension(0, 6).setCostPerDistance(1).build();
        /*
         * the algo requires some time and space to search for a valid solution. if you ommit a penalty-type, it probably throws an Exception once it cannot insert an envelope anymore
		 * thus, give it space by defining a penalty/shadow vehicle with higher variable and fixed costs to up the pressure to find solutions without penalty type
		 *
		 * it is important to give it the same typeId as the type you want to shadow
		 */


        class vehicleTemp{
            String vehicleTemp;
            Double CapacityTemp;
            Double tollFactor;
            Double extFactor;
            Integer containerFactor;
            Integer vehicleCost;
            Integer interest;
            Integer loanPeriod;
            Integer coolingTimeFactor;
            Integer amc;
            Integer insurance;
            Integer desielPrice;
            Integer milage;
            Integer speed;
            Integer coolingTime;
            Integer driverCost;
            Integer kmPerTyre;
            Integer costOfTyre;
            Integer noOfTyre;
            Integer repairCharge;

        }
/*

        BufferedReader reader1 = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/drivers.csv"))); //routing_Total_load_file_temp//trans_40//cost_eq_new_capacity_v2_nat_32_24
        String line1;
        boolean firstLine1 = true;
        while ((line1 = reader1.readLine()) != null) {
            if (firstLine1) {
                firstLine1 = false;
                continue;
            }

            String[] tokens1 = line1.split(",");

            VehicleType messengerType = VehicleTypeImpl.Builder.newInstance(tokens1[8])
                .addCapacityDimension(0, Integer.parseInt(tokens1[7]))
                .setMaxVelocity(Double.parseDouble(tokens1[9]))
                .setCostPerServiceTime(0)
                .setCostPerDistance(Double.parseDouble(tokens1[8]))//vehiclePerDistCost(60,Integer.parseInt(tokens[1]),Integer.parseInt(tokens[2]),80, Integer.parseInt(tokens[3]),Integer.parseInt(tokens[4]),Integer.parseInt(tokens[5]),Integer.parseInt(tokens[6]),Integer.parseInt(tokens[7]),Integer.parseInt(tokens[8]),Integer.parseInt(tokens[9]),Integer.parseInt(tokens[10]),Integer.parseInt(tokens[11]),Integer.parseInt(tokens[12]))) //
                .setFixedCost(0)
                .setCostPerTransportTime(0)
                .setCostPerWaitingTime(0)
                .build();

            VehicleImpl vehicle = VehicleImpl.Builder.newInstance(tokens1[0])  //i + "-" + keyTemp.toString() + "-" +
                //  .setLatestArrival(72*3600)
                .setStartLocation(vehicleMap.get(Location.Builder.newInstance().setCoordinate(Coordinate.newInstance(Double.parseDouble(tokens1[2]), Double.parseDouble(tokens1[3]))).build()))   //vehicleMap.get(keyTemp)//Location.Builder.newInstance().setCoordinate(Coordinate.newInstance(Double.parseDouble(tokens[2]), Double.parseDouble(tokens[3]))).build()
                .setEarliestStart(Double.parseDouble(tokens1[4]))
                .setReturnToDepot(false)
                .setType(messengerType)
                .build();

            problemBuilder.addVehicle(vehicle);
        }

*/



        for(Object keyTemp : vehicleMap.keySet()){


            //int i; i=0;
            BufferedReader reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/cost_eq_new_capacity_v2_nat_32_24.csv"))); //routing_Total_load_file_temp//trans_40//cost_eq_new_capacity_v2_nat_32_24
            String line;
            boolean firstLine = true;
            while ((line = reader.readLine()) != null) {
                if (firstLine) {
                    firstLine = false;
                    continue;
                }


                String[] tokens = line.split(",");
                for(int i=0;i<40;i++) {                //i<10

                    VehicleType messengerType = VehicleTypeImpl.Builder.newInstance(tokens[0])
                        .addCapacityDimension(0, Integer.parseInt(tokens[13]))
                        .setMaxVelocity(Double.parseDouble(tokens[2]))
                        .setCostPerServiceTime(0)
                        .setCostPerDistance(Double.parseDouble(tokens[14]))//vehiclePerDistCost(60,Integer.parseInt(tokens[1]),Integer.parseInt(tokens[2]),80, Integer.parseInt(tokens[3]),Integer.parseInt(tokens[4]),Integer.parseInt(tokens[5]),Integer.parseInt(tokens[6]),Integer.parseInt(tokens[7]),Integer.parseInt(tokens[8]),Integer.parseInt(tokens[9]),Integer.parseInt(tokens[10]),Integer.parseInt(tokens[11]),Integer.parseInt(tokens[12]))) //
                        .setFixedCost(0)
                        .setCostPerTransportTime(0)
                        .setCostPerWaitingTime(0)
                        .build();

                    VehicleImpl vehicle = VehicleImpl.Builder.newInstance(i + "-" + keyTemp.toString() + "-" + tokens[0])  //i + "-" + keyTemp.toString() + "-" +
                      //  .setLatestArrival(72*3600)
                        .setStartLocation(vehicleMap.get(keyTemp))   //vehicleMap.get(keyTemp)//Location.Builder.newInstance().setCoordinate(Coordinate.newInstance(Double.parseDouble(tokens[2]), Double.parseDouble(tokens[3]))).build()
                       // .setEarliestStart(Double.parseDouble(tokens[4]))
                        .setReturnToDepot(false)
                        .setType(messengerType)
                        .build();

                    problemBuilder.addVehicle(vehicle);

                    //i++;

                }


//                for(int i=5;i<10;i++) {                //i<10
//
//                    VehicleType messengerType = VehicleTypeImpl.Builder.newInstance(tokens[0])
//                        .addCapacityDimension(0, Integer.parseInt(tokens[15]))
//                        .setMaxVelocity(Double.parseDouble(tokens[2]))
//                        .setCostPerServiceTime(0)
//                        .setCostPerDistance(Double.parseDouble(tokens[14]))//vehiclePerDistCost(60,Integer.parseInt(tokens[1]),Integer.parseInt(tokens[2]),80, Integer.parseInt(tokens[3]),Integer.parseInt(tokens[4]),Integer.parseInt(tokens[5]),Integer.parseInt(tokens[6]),Integer.parseInt(tokens[7]),Integer.parseInt(tokens[8]),Integer.parseInt(tokens[9]),Integer.parseInt(tokens[10]),Integer.parseInt(tokens[11]),Integer.parseInt(tokens[12]))) //
//                        .setFixedCost(0)
//                        .setCostPerTransportTime(0)
//                        .setCostPerWaitingTime(0)
//                        .build();
//
//                    VehicleImpl vehicle = VehicleImpl.Builder.newInstance( i + "-" + keyTemp.toString() + "-" + tokens[0])
//                        //  .setLatestArrival(72*3600)
//                        .setStartLocation(vehicleMap.get(keyTemp))
//                        .setEarliestStart(99000)
//                        .setReturnToDepot(false)
//                        .setType(messengerType)
//                        .build();
//
//                    problemBuilder.addVehicle(vehicle);
//
//                }
            }
        }




//        for(Object keyTemp : vehicleMap.keySet()) {
//            for (int i = 0; i < 10; i++) {
//                VehicleType messengerTypeAir = VehicleTypeImpl.Builder.newInstance("Air")
//                    .addCapacityDimension(0, 1000000000)
//                    .setMaxVelocity(250)
//                    // .setCostPerServiceTime(0)
//                    .setCostPerDistance(2000000)
//                    //     .setFixedCost(vehicleFixedCost(Double.parseDouble(tokens[2]),Double.parseDouble(tokens[3])))
//                    //     .setCostPerTransportTime(vehiclePerTimeCost(Integer.parseInt(tokens[7]),Integer.parseInt(tokens[8]),Integer.parseInt(tokens[9]),Integer.parseInt(tokens[10]),Integer.parseInt(tokens[11]),Integer.parseInt(tokens[12])))
//                    //    .setCostPerWaitingTime(0)
//                    .build();
//
//                VehicleImpl vehicleAir = VehicleImpl.Builder.newInstance(i + "-" + keyTemp.toString() + "-Air")
//                    //  .setLatestArrival(72*3600)
//                    .setStartLocation(vehicleMap.get(keyTemp))
//                    .setReturnToDepot(false)
//                    .setType(messengerTypeAir)
//                    .build();
//
//                problemBuilder.addVehicle(vehicleAir);
//            }
//        }


//        BufferedReader reader = new BufferedReader(new FileReader(new File("/Users/megha.bhardwaj/jsprit/jsprit-examples/air_fleet_master.csv")));
//        String line;
//        boolean firstLine = true;
//        while ((line = reader.readLine()) != null) {
//            if (firstLine) {
//                firstLine = false;
//                continue;
//            }
//            String[] tokens = line.split(",");
//            //build your vehicle
//            VehicleType messengerType = VehicleTypeImpl.Builder.newInstance(tokens[4])
//                .addCapacityDimension(0, Integer.parseInt(tokens[5]))
//              //  .setProfile(tokens[4])
//      //          .setCostPerDistance(Double.parseDouble(tokens[6]))
//////                .setFixedCost(Integer.parseInt(tokens[6]))
//////                .setMaxVelocity(Double.parseDouble(tokens[10]))
//                .build();
//            //Integer.parseInt(tokens[6])
//        //    VehicleType messengerCostType = VehicleTypeImpl.VehicleCostParams.newInstance();
//
//            VehicleImpl vehicle = VehicleImpl.Builder.newInstance(tokens[1])
//               // .setEarliestStart(Double.parseDouble(tokens[6]))
//                .setLatestArrival(48*3600)
//                .setStartLocation(Location.Builder.newInstance().setCoordinate(Coordinate.newInstance(Double.parseDouble(tokens[2]), Double.parseDouble(tokens[3]))).build())
//                .setReturnToDepot(false)
//                .setType(messengerType)
////                .setEarliestStart(Double.parseDouble(tokens[6]))
//                .build();
//            problemBuilder.addVehicle(vehicle);
//        }
//       reader.close();
    }

    public static double vehicleFixedCost(double tollFactor, double extFactor){
        return tollFactor+extFactor;
    }

//    public static Integer vehiclePerTimeCost (Integer containerFactor, Integer vehicleCost, Integer interest, Integer loanPeriod, Integer amc, Integer insurance){
//
//        return containerFactor*vehicleCost*(1+interest)^(loanPeriod)*((12/10)/speed)/(loanPeriod*365)) * (1+(amc/10)+(insurance/10));
//    }

    public static Integer vehiclePerDistCost(Integer desielPrice, Integer milage, Integer speed, Integer driverCost, Integer kmPerTyre, Integer costOfTyre, Integer noOfTyre, Integer repairCharge, Integer containerFactor, Integer vehicleCost, Integer interest, Integer loanPeriod, Integer amc, Integer insurance){
        return (desielPrice/milage) + (((12/10)*driverCost)/(speed*(36/10))) + ((costOfTyre*noOfTyre)/kmPerTyre) +repairCharge +
        containerFactor*vehicleCost*((1+interest)^(loanPeriod))*(((12/10)/speed)/(loanPeriod*365)) * (1+(amc/10)+(insurance/10));
    }

}
