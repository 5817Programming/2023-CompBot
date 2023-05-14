package com.wcp.lib.geometry.HeavilyInspired;

import java.util.ArrayList;
import java.util.List;

import com.wcp.frc.subsystems.Swerve;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;


public class Node {
    public double x;
    public double y;
    Rotation2d holonomicRotation;
    public List < Node > neighbors;
  
    public Node(double x, double y) {
        this.x = x;
        this.y = y;
        holonomicRotation = Rotation2d.fromDegrees(0);
        this.neighbors = new ArrayList < > ();
    }
  
    public Node(double x, double y, Rotation2d holonomicRotation) {
        this.x = x;
        this.y = y;
        this.holonomicRotation = holonomicRotation;
        this.neighbors = new ArrayList < > ();
    }

    public Node(Swerve swerve){
      this.x = swerve.getPose().getTranslation().getX();
      this.y = swerve.getPose().getTranslation().getY(); 
      this.holonomicRotation = swerve.getPose().getRotation();
      this.neighbors = new ArrayList <>();
    }
  
    public Node(Translation2d coordinates, Rotation2d holonomicRotation) {
      this.x = coordinates.getX();
      this.y = coordinates.getY(); 
      this.holonomicRotation = holonomicRotation;
      this.neighbors = new ArrayList < > ();
    }



	public void addNeighbor(Node neighbor) {
        this.neighbors.add(neighbor);
    }
    public double getX(){
      return x;
    }
    public double getY(){
      return y;
    }
    public Translation2d getTranslation(){
      return new Translation2d(x,y);
    }
    public Rotation2d getHolRot(){
      return holonomicRotation;
    }
    public void setHolRot(double degree){
      this.holonomicRotation = Rotation2d.fromDegrees(degree);
    } 
  }
