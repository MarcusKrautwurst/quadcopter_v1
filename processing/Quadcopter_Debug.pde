import processing.serial.*;


Serial myPort; // The serial port
int xPos = 1; // horizontal position of the graph
int xPosLines = 1;
float var1Remap,var2Remap,var3Remap,var4Remap,altRemap;
float var1,var2,var3,var4;
float old1,old2,old3,old4;
int thickness = 200;
PFont f;
float y = 0.1;
float x = 0.1;
float z = 0.1;

void setup () 
{
  size(1280, 720);
  myPort = new Serial(this, Serial.list()[2], 115200);
  //myPort.write("go");
  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('-');
  // set inital background:
  background(20);
  f = createFont("Roboto",16,true); // Arial, 16 point, anti-aliasing on
}

void draw () 
{

}

void serialEvent (Serial myPort) 
{
  String input = myPort.readStringUntil('-');   
  if (input != null)
  {
   input = trim(input);
   input = input.replaceFirst("-", ""); 

   float inputs[] = float(split(input, ','));
   if(inputs.length == 13){
     float var1 = inputs[0];
     float var2 = inputs[1];
     float var3 = inputs[2];
     float var4 = inputs[3];
     float temp = inputs[4];
     float pitch = inputs[5];
     float roll = inputs[6];
     float p = inputs[7];
     float i = inputs[8];
     float d = inputs[9];
     float a = inputs[10];
     float aa = inputs[11];
     float mem = inputs[12];
     
    var1Remap = map(var1,0,180,height*0.2,(height*0.2)+thickness);
    var2Remap = map(var2,0,180,height*0.3,(height*0.3)+thickness);
    var3Remap = map(var3,0,180,height*0.4,(height*0.4)+thickness);
    var4Remap = map(var4,0,180,height*0.5,(height*0.5)+thickness);
    
    float errorsum = ((abs(128-var1)+abs(128-var2)+abs(128-var3)+abs(128-var4)));
    float error = round(errorsum/abs(errorsum-512)*100);
    
    //bottom ui
    fill(40);
    noStroke();
    textFont(f,20);
    rect(0, height*0.8, width, height);  

    if (xPos%10==0){
      stroke(30);
      line(xPos, 0, xPos, height);
    }
    
    if (xPos%50==0){
      stroke(50);
      line(xPos, 0, xPos, height);
    }
    
    
    String text1 = "FL: " + var1; 
    String text2 = "FR: " + var2;
    String text3 = "BL: " + var3;
    String text4 = "BR: " + var4;
    String text5 = "Temp.: " + temp + "°";
    String text_error = "Error: " + error + "%";
    String text_roll = "Setpoint Roll: " + roll + "%";
    String text_pitch = "Setpoint Pitch: " + pitch + "%";
    String textP = "P-Gain: " + p;
    String textI = "I-Gain: " + i;
    String textD = "D-Gain: " + d; 
    String textAlt = "Rel. Altitude: " + a;
    String textAbsAlt = "Abs. Altitude: " + aa;
    String textM = "Free Memory: " + mem;
    
    
    fill(255, 0, 0); 
    text(text1,0,height*0.85);
    
    fill(0, 255, 0); 
    text(text2,100,height*0.85);
    
    fill(0, 255, 255);    
    text(text3,0,height*0.93);
    
    fill(255, 255, 0);    
    text(text4,100,height*0.93);
    
    fill(255);    
    text(text5,250,height*0.85);   
    
    text(text_roll,400,height*0.85);   
    text(text_pitch,400,height*0.93);

    text(textP,700,height*0.85);   
    text(textI,700,height*0.89);   
    text(textD,700,height*0.93);  
    
   
    text(textAbsAlt,960,height*0.85);
    text(textAlt,960,height*0.89);
    text(textM,960,height*0.93);
    
    
    
    if (error<10){
      fill(30,255,30);
    } else { 
      fill(207,76,79);      
    }
        
    text(text_error,250,height*0.93);   
    
    // lower bars
    fill(40);
    stroke(20);
    rect(0,height*0.86,100,height*0.88);
    rect(110,height*0.86,210,height*0.88);
    rect(0,height*0.94,100,height*0.96);
    rect(110,height*0.94,210,height*0.96);
    
    fill(var1,0,0);
    rect(0,height*0.86,var1/1.8,height*0.88);
   
    fill(0,var2,0);
    rect(110,height*0.86,110+var2/1.8,height*0.88);
 
    fill(0,var3,var3);
    rect(0,height*0.94,var3/1.8,height*0.96);
    
    fill(var4,var4,0);
    rect(110,height*0.94,110+var4/1.8,height*0.96);

    // graph lines:
    stroke(var1, 0, 0);
    line(xPos, old1, xPos, var1Remap);

    stroke(0, var2, 0);
    line(xPos, old2, xPos, var2Remap);

    stroke(0, var3, var3);
    line(xPos, old3, xPos, var3Remap );
    
    stroke(var4, var4, 0);
    line(xPos, old4, xPos, var4Remap );     
     
    old1 = var1Remap;
    old2 = var2Remap;
    old3 = var3Remap;
    old4 = var4Remap;
    
    //sidebar
    fill(20);
    noStroke();
    rectMode(CORNER);
    rect(width-80, 0,width, height*0.8);
    
    rectMode(CORNERS);
    
    fill(255);
    rect(width-100,height,width-80,height-(aa*10)-100);
    
    fill(var1,0,0);
    rect(width-80,0,width-70,var1*2);
   
    fill(0,var2,0);
    rect(width-60,0,width-50,var2*2);
 
    fill(0,var3,var3);
    rect(width-40,0,width-30,var3*2);
    
    fill(var4,var4,0);
    rect(width-20,0,width-10,var4*2);
    
    
 
    


    // at the edge of the screen, go back to the beginning:
    if (xPos >= width-80) 
    {
      xPos = 0;
      background(0);
    } else {
      xPos++;}
      }
  }
}
