/**
 *  Force Directed Graph
 *
 *  Plots a force directed graph
 */
 
 static int Len = 11;
 static float kSpring = 0.25;
 static float SpringLen = 200;
 static float kDamp = 0.3;
 float distance;
 cPoint [] Points = new cPoint[2*Len];
 cEdge  [] Edges  = new cEdge[5*Len-4];
 PVector [] BonePoints;
 PVector Origin;
 cSimController SimControl;
 cDataList SignalData;
 int index = 0;
 int Delay = 0;
 int DelayMax = 1000;
 
 StringList BoneData = new StringList();
 
 void setup() {
   println(new PVector(1,0).heading());
   println(new PVector(1,1).heading());
   println(new PVector(1,-1).heading());
   fullScreen();

   Origin = new PVector(300, height/2);

   // Create a new simulation controller
   SimControl = new cSimController(10000.0, kDamp, 0.05);
   // Load the neuron data from file
   SignalData = LoadSigFile("./Move.sig");
      
   for (int i = 0; i < 2*Len; i = i+2) {
     Points[i] = new cPoint((i+4)*40,height/2-25,0.001,1.0, SimControl);
     Points[i+1] = new cPoint((i+4)*40,height/2+25,0.001,1.0, SimControl);
     // Edges
     Edges[i/2] = new cEdge(i,i + 1,0.5,SpringLen);
   }
   
   for (int i = 0; i < 2*Len-2; i = i + 2) {
     Edges[Len + i] = new cEdge(i,i + 2,kSpring,SpringLen);
     Edges[Len + i + 1] = new cEdge(i + 1,i + 3,kSpring,SpringLen);
   }
   
   for (int i = 0; i < 2*Len - 2; i = i + 2) {
     Edges[3*Len - 2 + i] = new cEdge(i,i + 3,0.5,sqrt(2*sq(SpringLen)));
     Edges[3*Len - 2 + i + 1] = new cEdge(i + 1,i + 2,0.5,sqrt(2*sq(SpringLen))); //<>//
   }
   
   for (int i = 0; i < 100; i++) {
     for (cPoint Point : Points) {
       // Clear the forces
       Point.ClearForce();
     }
     
     for (cEdge Edge : Edges) {
       Edge.AddAttractionForce(Points);
     }
     // For each point in the list...
     for (cPoint Point : Points) {
       // Calculate new forces
       Point.AddRepulsionForces(Points);
       // Draw the points
       Point.ApplyForce();
     } 
   }
 }
 
 int hold = 0;
 
 void draw() {
   float [] BoneAngles;
   PVector BoneOffset;
   cData DataEntry;
   // Clear the screen
   background(255);
   
   DataEntry = SignalData.GetcData(index); //<>//
   
   CentreCamera(Points);
   
   ActuateMuscles(Edges, DataEntry);
   
   index = index + 5;  //<>//
   
   for (cPoint Point : Points) {
     // Clear the forces
     Point.ClearForce();
   }
   
   for (cEdge Edge : Edges) {
     Edge.AddAttractionForce(Points);
     Edge.DrawEdge(Points);
   }
   // For each point in the list...
   for (cPoint Point : Points) {
     // Calculate new forces
     Point.AddRepulsionForces(Points);
     // Draw the points
     Point.DrawPoint();
     Point.DrawForce();
     // Apply forces
     //if ( Points[0] != Point ) {
       Point.ApplyForce();
     //}
   }
   
   BonePoints = CalcBonePosition(Points);
   BoneAngles = CalcBoneAngles(BonePoints);
   BoneOffset = CalcOriginOffset(BonePoints);
   
   AppendBoneData(BoneData, BoneAngles,BoneOffset);
   
   DrawBoneJoints(BonePoints);
   DrawOrigin();
   
   if (index >= SignalData.DM.size()) {
     hold++;
     index = SignalData.DM.size() - 1;
   }
   
   if (index + hold >= SignalData.DM.size() + 300) {
     // Save the data
     SaveBoneData(BoneData, "BoneOutputs.csv");
     print("Done!");
     noLoop();
   }
   while(true){};
 }
 
 void ActuateMuscles(cEdge [] EdgeList, cData MuscleData) {
   float kMuscle = 0.01;
   
   float DCurrentValue;
   float DTarget;
   float DDifference;
   float DNewValue;
   
   float VCurrentValue;
   float VTarget;
   float VDifference;
   float VNewValue;  
   
   for (int i = 0; i<2*Len-2; i=i+2) {
     DCurrentValue = EdgeList[Len+i].Length;
     DTarget = SpringLen*int(MuscleData.DM.charAt(i/2) == '0');
     DDifference = DTarget - DCurrentValue;
     DNewValue = kMuscle*DDifference + DCurrentValue;  
     
     EdgeList[Len+i].SetLength(DNewValue);
     
     VCurrentValue = EdgeList[Len+i+1].Length;
     VTarget = SpringLen*int(MuscleData.VM.charAt(i/2) == '0');
     VDifference = VTarget - VCurrentValue;
     VNewValue = kMuscle*VDifference + VCurrentValue;  
     
     EdgeList[Len+i+1].SetLength(VNewValue);
   }
 }
 
 PVector[] CalcBonePosition(cPoint Point_List[]) {
   PVector [] BonePins = new PVector[Len];
   PVector    MidPoint = new PVector();
   for (int i=0; i<Len; i++) {
     // Find the location between each of the point pairs
     MidPoint = PVector.add(Point_List[2*i].Position, Point_List[2*i+1].Position);
     MidPoint.mult(0.5);
     // Add the new midpoint to the bone pin list
     BonePins[i] = MidPoint;
   }
   return BonePins;
 }
 
 float[] CalcBoneAngles(PVector BonePins[]) {
   float [] Angles = new float[Len-1];
   PVector BoneA;
   PVector BoneB;
   
   // Find the angle made with the origin
   BoneA = new PVector(1,0);
   BoneB = PVector.sub(BonePins[1], BonePins[0]);
   Angles[0] = BoneB.heading() - BoneA.heading();
   
   // Find the angle made by internal bones
   for (int i = 0; i<Len-2; i++) {
     BoneA = PVector.sub(BonePins[i+1], BonePins[i]);
     BoneB = PVector.sub(BonePins[i+2], BonePins[i+1]);
     Angles[i+1] = BoneB.heading() - BoneA.heading();
   }
  
   return Angles;
 }
 
 PVector CalcOriginOffset(PVector BonePins[]) {
   return PVector.sub(BonePins[0], Origin);
 }

 void AppendBoneData(StringList BoneData, float BoneAngles[], PVector BoneOffset) {
   String Row = "";
   for (float Angle : BoneAngles) {
     Row = Row + str(Angle) + ",";
   }
   Row = Row + str(BoneOffset.x) + "," + str(BoneOffset.y);
   BoneData.append(Row);
 }
 
 void SaveBoneData(StringList BoneData, String FileName) {
   // Load the data as an array of strings
   saveStrings(FileName, BoneData.array());
 }
 
 void DrawBoneJoints(PVector BonePins[]){
     // Constants for drawing
     float Size = 25.0;
     // Prepare line for drawing a point
     strokeWeight(2);
     stroke(160,160,160);
     fill(200,200,200);
     for (PVector Position : BonePins) {
       // Draw a circle at the points location
       ellipse(Position.x, Position.y, Size, Size);
     }
 }
 
 void DrawOrigin() {
     // Constants for drawing
     float Size = 25.0;
     // Prepare line for drawing a point
     strokeWeight(2);
     stroke(245,118,42);
     fill(245,165,65);
     ellipse(Origin.x, Origin.y, Size, Size);
 }
 
 cDataList LoadSigFile(String FileName) {
   String [] DataRows;
   String [] SplitRow;
   cDataList DataList = new cDataList();
   // Load the data as an array of strings
   DataRows = loadStrings(FileName);
   // For each row in the data, extract the information.
   for (String Row : DataRows) {
     Row.trim();
     SplitRow = split(Row, ", ");
     DataList.AppendData(SplitRow[0],SplitRow[1],SplitRow[2],SplitRow[3]);
   }
   return DataList;
 }
 
 PVector CalcCentre(cPoint Point_List[]) {
   PVector PosSum;
   // Start at 0,0
   PosSum = new PVector(0,0);
   // Add all point position up
   for (cPoint Point : Point_List) {
     PosSum.add(Point.Position);
   }
   // Divide by total number of points
   PosSum.div(Point_List.length);
   // Return centre point
   return PosSum;
 }
 
 void CentreCamera(cPoint Point_List[]) {
   PVector Centre;
   PVector Shift;
   // Calculate centre of network
   Centre = CalcCentre(Point_List);
   // Offset by centre of screen
   Shift = PVector.sub(new PVector(width/2,height/2), Centre);
   for (cPoint Point : Point_List) {
     Point.Position.add(Shift);
   }
 }
 
 // === Network classes ===
 
 public class cData {
   public String DM;
   public String VM;
   public String Fwd;
   public String Coil;
 }
 
 public class cDataList {
   public StringList DM;
   public StringList VM;
   public StringList Fwd;
   public StringList Coil;
   
   cDataList() {
     DM = new StringList();
     VM = new StringList();
     Fwd = new StringList();
     Coil = new StringList();
   }
   
   cData GetcData(int id) {
     cData Data = new cData();
     // Pass the data into a cData class
     Data.DM   = DM.get(id);
     Data.VM   = VM.get(id);
     Data.Fwd  = Fwd.get(id);
     Data.Coil = Coil.get(id);
     // Return the extraced data
     return Data;
   }
   
   void ClearAll() {
     DM.clear();
     VM.clear();
     Fwd.clear();
     Coil.clear();
   }
   
   void AppendData(String DataDM, String DataVM, String DataFwd, String DataCoil) {
     DM.append(DataDM);
     VM.append(DataVM);
     Fwd.append(DataFwd);
     Coil.append(DataCoil);
   }
 }
 
 class cSimController {
   float kCoulomb;
   float dT;
   float Damp;
   
   cSimController(float kC, float kd, float deltaT) {
     kCoulomb = kC;
     Damp = kd;
     dT = deltaT;
   }
 }
 
 class cPoint {
   // Base class properties
   PVector Position;
   PVector Velocity;
   float Mass;
   float Charge;
   PVector Force;
   // Simulation globals
   cSimController Sim;
   
   cPoint(float x, float y, float m, float q, cSimController Controller) {
     // Initialise a point
     Position = new PVector(x, y);
     Mass = m;
     Charge = q;
     // Create the points without any force applied.
     Force = new PVector(0,0);
     Velocity = new PVector(0,0);
     // Simulation Globals
     Sim = Controller;
   }
   
   void ClearForce() {
     // Clear the applied forces ready for the next calculation
     Force = new PVector(0,0);
   }
   
   void AddRepulsionForces(cPoint Point_List[]) {
     float Distance;
     float Force;
     PVector Direction;
     PVector Repulsion;
     // Cycle through each of the points in the point list
     for (cPoint Point : Point_List){
       // Check that the point selected is not this point
       if (this != Point) {
         Distance = Position.dist(Point.Position);
         Force = (this.Sim.kCoulomb * this.Charge * Point.Charge)/sq(Distance);
         // Find the direction of the force
         Direction = PVector.sub(this.Position, Point.Position);
         // Normalise the direction
         Direction.normalize();
         // Calculate the repulsion vector
         Repulsion = PVector.mult(Direction, Force);
         // Apply repulsion to point
         this.Force.add(Repulsion);
       }
     }
   }
   
   void ApplyForce() {
     PVector Acceleration;
     // Calculate acceleration using F=mA
     Acceleration = PVector.div(Force, Mass);
     // Calculate the resulting velocity using V = V + A*dT 
     Velocity.add(PVector.mult(Acceleration, Sim.dT));
     // Apply damping F = kV
     Velocity.mult(Sim.Damp);
     // Apply velocity to point
     Position.add(PVector.mult(Velocity, Sim.dT));
     
   }
   
   // --- Draw functions for debug ---
   
   void DrawForce() {
     // Length Scale for drawn line
     float LengthScale = 10.0;
     // Prepare line for drawing a force
     strokeWeight(2);
     stroke(255,0,0);
     // Draw the force as a line
     line(Position.x,Position.y,Position.x+LengthScale*Force.x,Position.y+LengthScale*Force.y);
   }
   
   void DrawPoint() {
     // Constants for drawing
     float Size = 25.0;
     // Prepare line for drawing a point
     strokeWeight(2);
     stroke(75,75,180);
     fill(125,125,255);
     // Draw a circle at the points location
     ellipse(Position.x, Position.y, Size, Size);
   }
 }
 
 
 
 class cEdge {
   int Start_Point, End_Point;
   float Strength;
   float Length;
   
   cEdge(int Start, int End, float k, float L) {
     // Initialise an edge
     Start_Point = Start;
     End_Point = End;
     Strength = k;
     Length = L; 
   }
   
   float CalcAttractionForce(cPoint Point_List[]) {
     float F;
     float Distance;
     float Extension;
     // Calculate the euclidean distance between the two points
     Distance = Point_List[Start_Point].Position.dist(Point_List[End_Point].Position);
     // Calculate the extension
     Extension = Distance - Length; 
     // Calculate attraction force using Hooke's law
     F = -Strength *  Extension;
     // Return the force
     return F;
   }
   
   void AddAttractionForce(cPoint Point_List[]) {
     float Force;
     PVector Direction;
     PVector Attraction;
     // Calculate the force applied by this edge
     Force = CalcAttractionForce(Point_List);     
     // Find the direction of the edge (start to end)
     Direction = PVector.sub(Point_List[Start_Point].Position, Point_List[End_Point].Position);
     // Normalise the direction
     Direction.normalize();
     // Calculate the attraction vector
     Attraction = PVector.mult(Direction, Force);
     // Apply the attraction force to the two points
     Point_List[Start_Point].Force.add(Attraction);
     Point_List[End_Point].Force.sub(Attraction);
   }
   
   void SetLength(float newLength) {
     Length = newLength;
   }
   
   void DrawEdge(cPoint Point_List[]) {
     // Colour for muscle activation
     int Colour = 255 - int(Length/SpringLen * 255);
     // Width for muscle activation
     float Width = 1 - Length/SpringLen;
     if (Width>1.0) {Width = 1.0;}
     // Prepare line for drawing an edge
     strokeWeight(Width*3+3);
     stroke(Colour,75,75);
     // Draw a circle at the points location
     line(Point_List[Start_Point].Position.x, Point_List[Start_Point].Position.y, Point_List[End_Point].Position.x, Point_List[End_Point].Position.y);
   }
 }
 
