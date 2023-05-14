// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import javax.xml.namespace.QName;

import com.wcp.frc.subsystems.Requests.Request;
import com.wcp.frc.subsystems.Requests.RequestList;

/** Add your docs here. */
public class SuperStructure extends Subsystem{
    public Swerve swerve;
    public Intake intake;
    public Lights lights;
    public Arm arm;
    public SideElevator sideElevator;
    public Vision vision;

    private ArrayList<RequestList> queuedRequests;

    public SuperStructure(){
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        lights = Lights.getInstance();
        arm = Arm.getInstance();
        intake = Intake.getInstance();
        sideElevator = SideElevator.getInstance();
        vision = Vision.getInstance();

        queuedRequests = new ArrayList<>();


    }

    public static SuperStructure instance = null;

    public static SuperStructure getInstance(){
        if(instance == null)
            instance = new SuperStructure();
        return instance;
    }

    private RequestList activeRequests;
    Request currentRequest;

    private boolean newRequests;
    private boolean activeRequestsComplete;
    private boolean allRequestsComplete;
    private boolean requestsCompleted(){return allRequestsComplete;}

    private void setActiveRequests(RequestList requests){
        activeRequests = requests;
        newRequests = true;
        activeRequestsComplete = false;
        allRequestsComplete = false;
    }

    private void setQueueRequests(RequestList r){
        queuedRequests.clear();
        queuedRequests.add(r);
    }

    private void setQueueRequests(List<RequestList> requests){
        queuedRequests.clear();
        queuedRequests = new ArrayList<>(requests.size());
        for(RequestList r: requests){
            queuedRequests.add(r);
        }
    }

    private void request(Request r){
        setActiveRequests(new RequestList(Arrays.asList(r),false));
        setQueueRequests(new RequestList());
    }
    private void request(Request r, Request q){
        setActiveRequests(new RequestList(Arrays.asList(r),false));
        setQueueRequests(new RequestList(Arrays.asList(q), false));
    }

    private void request(RequestList r){
        setActiveRequests(r);
        setQueueRequests(new RequestList());
    }
    
    private void request(RequestList r, RequestList q){
        setActiveRequests(r);
        setQueueRequests(q);
    }
    
    public void addActiveRequests(Request r){
        activeRequests.add(r);
        newRequests = true;
        activeRequestsComplete = false;
        allRequestsComplete = false;
    }   
    public void addForemostActiveRequest(Request request){
		activeRequests.addToForefront(request);
		newRequests = true;
		activeRequestsComplete = false;
		activeRequestsComplete = false;
	}
	
	public void queue(Request request){
		queuedRequests.add(new RequestList(Arrays.asList(request), false));
	}
	
	public void queue(RequestList list){
		queuedRequests.add(list);
	}
	
	public void replaceQueue(Request request){
		setQueueRequests(new RequestList(Arrays.asList(request), false));
	}
	
	public void replaceQueue(RequestList list){
		setQueueRequests(list);
	}
	
	public void replaceQueue(List<RequestList> lists){
		setQueueRequests(lists);
	}


    public void update(){
        synchronized(SuperStructure.this){
        
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
                    currentRequest = activeRequests.remove();//takes first thing outr of list and into variable
                    currentRequest.act();
                    newRequests = false;
                }
            }
        }
        if(activeRequests.isParallel()){
            boolean done = true;
            for(Request request : activeRequests.getRequests()){
                done &= request.isFinished();//if done and request is finsihed
            }
            activeRequestsComplete = done;
        }else if(currentRequest.isFinished()){
            if(activeRequests.isEmpty()){
                activeRequestsComplete = true;
            }else if(activeRequests.getRequests().get(0).allowed()){
                newRequests = true;
                activeRequestsComplete = true;
            }
        }else{
            if(!queuedRequests.isEmpty()){
                setActiveRequests(queuedRequests.remove(0));
            }else{
                allRequestsComplete = true;
            }
        }
    }
    }

    
    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }
    @Override
    public void stop() {
        
        
    }
}
