
import processing.serial.*;

/**
 * RollingGraph
 * This sketch makes ise of the RollingLine2DTrace object to
 * draw a dynamically updated plot.
 */

import org.gwoptics.graphics.graph2D.Graph2D;
import org.gwoptics.graphics.graph2D.traces.ILine2DEquation;
import org.gwoptics.graphics.graph2D.traces.RollingLine2DTrace;

Serial myPort;
String val, line;


class eq implements ILine2DEquation{
  public double computePoint(double x,int pos) {
          val =  myPort.readStringUntil('\n');
          //if (val.indexOf('t') != -1) {
          // val = val.substring(0, val.indexOf('t')); 
          //}
          if(val != null) {
            System.out.println(val);
            return Float.parseFloat(val);
          }
          else {
            System.out.println(val);
            //return Float.parseFloat(val.substring(0, (val.indexOf('t'))));
            return 0;
          }
  }    
}

//class eq2 implements ILine2DEquation{
//  public double computePoint(double x,int pos) {
//         return 21.1;
//  }    
//}

//class eq3 implements ILine2DEquation{
//  public double computePoint(double x,int pos) {
//    if(mousePressed)
//      return 400;
//    else
//      return 0;
//  }    
//}

RollingLine2DTrace r; // add r2,r3
Graph2D g;
  
void setup(){
  String portName = Serial.list()[1];
  myPort = new Serial(this, portName, 115200);
  myPort.clear();
  size(1100,800);
  
  r  = new RollingLine2DTrace(new eq(), 10, 0.01f);
  r.setTraceColour(255, 0, 0);
  r.setLineWidth(1);
  
  //r2 = new RollingLine2DTrace(new eq2(),100,0.1f);
  //r2.setTraceColour(255, 255, 0);
  
  //r3 = new RollingLine2DTrace(new eq3(),100,0.1f);
  //r3.setTraceColour(0, 0, 255);
   
  g = new Graph2D(this, 900, 500, false);
  g.setYAxisMax(90);
  g.setYAxisMin(-90);
  g.addTrace(r);
  //g.addTrace(r2);
  //g.addTrace(r3);
  g.position.y = 50;
  g.position.x = 100;
  g.setYAxisTickSpacing(10);
  g.setXAxisMax(10f);
}

void draw(){
  //while (myPort.available() > 0) {
  //  int inByte = myPort.read();
  //  println(inByte);
  //}
  background(200);
  g.draw();
}