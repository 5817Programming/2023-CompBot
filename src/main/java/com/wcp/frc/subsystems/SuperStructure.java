// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.Requests.RequestList;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.HeavilyInspired.Node;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SuperStructure extends Subsystem {
    public Swerve swerve;
    public Intake intake;
    public Lights lights;
    public Elevator elevator;
    public Arm arm;
    public SideElevator sideElevator;
    public Vision vision;

    private ArrayList<RequestList> queuedRequests;

    public SuperStructure() {
        elevator = Elevator.getInstance();
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        lights = Lights.getInstance();
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        sideElevator = SideElevator.getInstance();
        vision = Vision.getInstance();
        idleState();
        queuedRequests = new ArrayList<>();

    }


    public static SuperStructure instance = null;

    public static SuperStructure getInstance() {
        if (instance == null)
            instance = new SuperStructure();
        return instance;
    }
    private RequestList activeRequests;
    private RequestList idleRequests;
    private Translation2d swerveControls = new Translation2d();
    private double swerveRotation = 0;
    Request currentRequest;


    private boolean newRequests;
    private boolean activeRequestsComplete = true;
    private boolean allRequestsComplete;

    private boolean requestsCompleted() {
        return activeRequestsComplete;
    }

    State currentState = State.ZERO;
    PreState currentUnprocessedState = PreState.ZERO;

    boolean cube = true;
    
    public enum GameState{
        GETPIECE,
        SCORE,
        CHARGE;
    }

    public enum PreState {
        HIGH,
        MID,
        LOW,
        ZERO;

    }

    public enum State {
        HIGHCONE(Elevator.State.HIGHCONE, SideElevator.State.HIGHCONE, Arm.State.HIGHCONE),
        HIGHCUBE(Elevator.State.HIGHCUBE, SideElevator.State.HIGHCUBE, Arm.State.HIGHCUBE),
        MIDCONE(Elevator.State.MIDCONE, SideElevator.State.MIDCONE, Arm.State.MIDCONE),
        MIDCUBE(Elevator.State.MIDCUBE, SideElevator.State.MIDCUBE, Arm.State.MIDCUBE),
        LOWCONE(Elevator.State.LOWCONE, SideElevator.State.LOWCONE, Arm.State.LOWCONE),
        LOWCUBE(Elevator.State.LOWCUBE, SideElevator.State.LOWCUBE, Arm.State.LOWCUBE),
        HUMANCONE(Elevator.State.HOOMANCONE, SideElevator.State.HOOMANCONE, Arm.State.HOOMANCONE),
        HUMANCUBE(Elevator.State.HOOMANCUBE, SideElevator.State.HOOMANCUBE, Arm.State.HOOMANCUBE),
        CHUTE(Elevator.State.CHUTE, SideElevator.State.CHUTE, Arm.State.CHUTE),
        ZERO(Elevator.State.ZERO, SideElevator.State.ZERO, Arm.State.ZERO);

        Elevator.State elevatorState = Elevator.State.ZERO;
        Arm.State armState = Arm.State.ZERO;
        SideElevator.State sideElevatorState = SideElevator.State.ZERO;

        private State(Elevator.State eState, SideElevator.State sState, Arm.State aState) {
            this.elevatorState = eState;
            this.sideElevatorState = sState;
            this.armState = aState;

        }

    }

    private void setActiveRequests(RequestList requests) {
        activeRequests = requests;
        newRequests = true;
        activeRequestsComplete = false;
        allRequestsComplete = false;
    }

    private void setQueueRequests(RequestList r) {
        queuedRequests.clear();
        queuedRequests.add(r);
    }

    private void setQueueRequests(List<RequestList> requests) {
        queuedRequests.clear();
        queuedRequests = new ArrayList<>(requests.size());
        for (RequestList r : requests) {
            queuedRequests.add(r);
        }
    }

    private void request(Request r) {
        setActiveRequests(new RequestList(Arrays.asList(r), false));
        setQueueRequests(new RequestList());
    }

    private void request(Request r, Request q) {
        setActiveRequests(new RequestList(Arrays.asList(r), false));
        setQueueRequests(new RequestList(Arrays.asList(q), false));
    }

    private void request(RequestList r) {
        setActiveRequests(r);
        setQueueRequests(new RequestList());
    }

    private void request(RequestList r, RequestList q) {
        setActiveRequests(r);
        setQueueRequests(q);
    }

    public void addActiveRequests(Request r) {
        activeRequests.add(r);
        newRequests = true;
        activeRequestsComplete = false;
        allRequestsComplete = false;
    }

    public void addForemostActiveRequest(Request request) {
        activeRequests.addToForefront(request);
        newRequests = true;
        activeRequestsComplete = false;
        activeRequestsComplete = false;
    }

    public void queue(Request request) {
        queuedRequests.add(new RequestList(Arrays.asList(request), false));
    }



    public void queue(RequestList list) {
        queuedRequests.add(list);
    }

    public void replaceQueue(Request request) {
        setQueueRequests(new RequestList(Arrays.asList(request), false));
    }
    public void clearQueues(){
        setActiveRequests(new RequestList());
        setQueueRequests(new RequestList());
    }

    public void replaceQueue(RequestList list) {
        setQueueRequests(list);
    }

    public void replaceQueue(List<RequestList> lists) {
        setQueueRequests(lists);
    }

    public void requestSwerveInput(Translation2d x, double r) {
        this.swerveControls = x;
        this.swerveRotation = r;
    }

    public void processState() {
        switch (currentUnprocessedState) {
            case HIGH:
                if (cube)
                    currentState = State.HIGHCUBE;
                else
                    currentState = State.HIGHCONE;
            break;

            case MID:
                if (cube)
                    currentState = State.HIGHCUBE;
                else
                    currentState = State.HIGHCONE;
            break;

            case LOW:
                if (cube)
                    currentState = State.LOWCUBE;
                else
                    currentState = State.LOWCONE;
            break;

            case ZERO:
                currentState = State.ZERO;
            break;
        }
    }

    public void setRequestHeight(PreState state) {
        currentUnprocessedState = state;
    }

    public void setPiece(boolean cube) {
        this.cube = cube;
        processState();
    }

    public void setHeight(PreState newState) {
        currentUnprocessedState = newState;
    }

    public void update() {
        synchronized (SuperStructure.this) {

            if(!activeRequestsComplete){
                if(newRequests){
                    if(activeRequests.isParallel()){
                        boolean allActivated = true;
                        for(Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator.hasNext();){
                            Request request = iterator.next();
                            boolean allowed = request.allowed();
                            allActivated &= allowed;
                            if(allowed) request.act();
                        }
                        newRequests = !allActivated;
                    }else{
                        if(activeRequests.isEmpty()){
                            activeRequestsComplete = true;
                            return;
                        }
                        currentRequest = activeRequests.remove();
                        currentRequest.act();
                        currentRequest.initialize();
                        newRequests = false;
                    }
                }
                if(activeRequests.isParallel()){
                    boolean done = true;
                    for(Request request : activeRequests.getRequests()){
                        done &= request.isFinished();
                    }
                    activeRequestsComplete = done;
                }else if(currentRequest.isFinished()){
                        if(activeRequests.isEmpty()){
                            activeRequestsComplete = true;
                        }else if(activeRequests.getRequests().get(0).allowed()){
                            newRequests = true;
                            activeRequestsComplete = false;
                        }
                }else{
                    currentRequest.act();
                }
            }else{
                if(!queuedRequests.isEmpty()){
                    setActiveRequests(queuedRequests.remove(0));
                }else{
                    idleState();
                    for(Iterator<Request> iterator = idleRequests.getRequests().iterator(); iterator.hasNext();){
                        Request request = iterator.next();
                        boolean allowed = request.allowed();
                        if(allowed) request.act();
                    }
                    activeRequestsComplete = true;
                }
            }

            } 
        }
    

    public void aimState(boolean snapUp, boolean snapDown) {
        RequestList request = new RequestList(Arrays.asList(
                arm.stateRequest(Arm.State.ZERO),
                sideElevator.stateRequest(SideElevator.State.ZERO),
                elevator.idleRequest()),
                true);
        RequestList queue = new RequestList(Arrays.asList(
                swerve.aimStateRequest(snapUp, snapDown),
                sideElevator.stateRequest(currentState.sideElevatorState),
                arm.stateRequest(currentState.armState),
                elevator.stateRequest(currentState.elevatorState),
                intake.percentOutputRequest(!cube),
                intake.stopIntakeRequest(),
                swerve.openLoopRequest(swerveControls, swerveRotation)
                ),
                 false);
            request(request,queue);
    }

    public void balanceState(){
        request(swerve.balanceRequest());
    } 

    public void toChuteState(){
        RequestList request = new RequestList(Arrays.asList(
            elevator.idleRequest(),
            sideElevator.stateRequest(SideElevator.State.ZERO),
            arm.stateRequest(Arm.State.ZERO))
            , true);
        RequestList queue = new RequestList(Arrays.asList(
            swerve.goToChuteRequest(),
            arm.stateRequest(Arm.State.CHUTE),
            intake.percentOutputRequest(!cube),
            intake.waitUntilIntakedPieceRequest(),
            intake.stopIntakeRequest())
            ,false);
        request(request, queue);
    }

    public void objectTargetState(){
        RequestList request = new RequestList(Arrays.asList(
            elevator.stateRequest(Elevator.State.PICKUP),
            arm.stateRequest(Arm.State.PICKUP),
            sideElevator.stateRequest(SideElevator.State.PICKUP),
            intake.percentOutputRequest(1,cube)
        ),true);
        RequestList queue = new RequestList(Arrays.asList(
            swerve.objectTartgetRequest(),
            intake.waitUntilIntakedPieceRequest(),
            intake.stopIntakeRequest(),
            intake.intakeBrakeRequest(),
            swerve.openLoopRequest(swerveControls, swerveRotation)
        ),false);
        request(request,queue);
    }
    public void targetNodeState(int node){
        RequestList request = new RequestList(Arrays.asList(
                arm.stateRequest(Arm.State.ZERO),
                sideElevator.stateRequest(SideElevator.State.ZERO),
                elevator.stateRequest(Elevator.State.ZERO)),
                true);
        RequestList queue = new RequestList(Arrays.asList(
                swerve.goToNodeRequest(node),
                sideElevator.stateRequest(currentState.sideElevatorState),
                arm.stateRequest(currentState.armState),
                elevator.stateRequest(currentState.elevatorState),
                intake.percentOutputRequest(!cube),
                swerve.openLoopRequest(swerveControls, swerveRotation)
                ),
                 false);
            request(request,queue);
    }

    public void idleState() {
        RequestList request = new RequestList(Arrays.asList(
            swerve.openLoopRequest(swerveControls, swerveRotation),
            elevator.idleRequest(),
            sideElevator.stateRequest(SideElevator.State.ZERO),
            arm.stateRequest(Arm.State.ZERO),
            intake.intakeBrakeRequest(),
            lights.lightRequest(cube ? Lights.State.CUBE: Lights.State.CONE)
        ), false);
        idleRequests = request;
    }

    public void scoreState(PreState state, boolean cube){
        setPiece(cube);
        RequestList request = new RequestList(Arrays.asList(
            swerve.setStateRequest(Swerve.State.OFF),
            sideElevator.stateRequest(currentState.sideElevatorState),
            arm.stateRequest(currentState.armState),
            elevator.stateRequest(currentState.elevatorState)
            ),false);
        RequestList queue = new RequestList(Arrays.asList(
            intake.percentOutputRequest(!this.cube),
            intake.stopIntakeRequest()
            ),false);
        request(request, queue);
    }
    public void scoreState(State state){
        currentState = state;
        RequestList request = new RequestList(Arrays.asList(
            swerve.setStateRequest(Swerve.State.OFF),
            sideElevator.stateRequest(currentState.sideElevatorState),
            arm.stateRequest(currentState.armState),
            elevator.stateRequest(currentState.elevatorState)
            ),false);
        RequestList queue = new RequestList(Arrays.asList(
            intake.percentOutputRequest(!this.cube),
            intake.stopIntakeRequest()
            ),false);
        request(request, queue);
        
    }

    public void trajectoryState(int node){
        RequestList request = new RequestList(Arrays.asList(
            swerve.generateTrajectoryRequest(node),
            swerve.startPathRequest(4, true)
        ),false);
        request(request);
    }

    public void trajectoryState(Node node){
        RequestList request = new RequestList(Arrays.asList(
            swerve.generateTrajectoryRequest(node),
            swerve.startPathRequest(4, true)
        ),false);
       request(request);
    }
  
    public void scoreState(PreState state){
        processState();
        RequestList request = new RequestList(Arrays.asList(
            swerve.setStateRequest(Swerve.State.OFF),
            sideElevator.stateRequest(currentState.sideElevatorState),
            arm.stateRequest(currentState.armState),
            elevator.stateRequest(currentState.elevatorState)
            ),false);
        RequestList queue = new RequestList(Arrays.asList(
            intake.percentOutputRequest(!this.cube),
            intake.stopIntakeRequest()
            ),false);
        request(request, queue);
    }
    public void scoreState(){
        RequestList request = new RequestList(Arrays.asList(
            swerve.setStateRequest(Swerve.State.OFF),
            sideElevator.stateRequest(currentState.sideElevatorState),
            arm.stateRequest(currentState.armState),
            elevator.stateRequest(currentState.elevatorState)
            ),false);
        RequestList queue = new RequestList(Arrays.asList(
            intake.percentOutputRequest(!this.cube),
            intake.stopIntakeRequest()
            ),false);
        request(request, queue);
    }


  
    public Request waitRequest(double waitTime){
        return new Request() {
            Timer timer = new Timer();
            @Override
                public boolean isFinished(){
                    if(timer.hasElapsed(waitTime)){
                        timer.reset();
                        return true;
                    }
                    else return timer.hasElapsed(waitTime);
                }
        

            @Override
                public void act() {
                    timer.start();                    
                }};
    }

    public void actOnGameState(){
        
    }

    @Override
    public void outputTelemetry() {
        Logger.getInstance().recordOutput("RequestsCompleted", requestsCompleted());
    }

    @Override
    public void stop() {

    }
}
