within BuildingMpc.Examples.MPCs;
model MpcCase900GEOTABS
  extends UnitTests.MPC.BaseClasses.Mpc(
    final nOut=16,
    final nOpt=4,
    final nSta=150,
    final nMeas=0,
    final controlTimeStep=3600,
    final nModCorCoeff=21,
    final name= "Case900GEOTABS");
  Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,3, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,4, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,5, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround1 = getOutput(tableID,6, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround2 = getOutput(tableID,7, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround3 = getOutput(tableID,8, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround4 = getOutput(tableID,9, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround5 = getOutput(tableID,10, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround6 = getOutput(tableID,11, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround7 = getOutput(tableID,12, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround8 = getOutput(tableID,13, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround9 = getOutput(tableID,14, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput TGround10 = getOutput(tableID,15, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,16, time)
    annotation (Placement(transformation(extent={{96,50},{116,70}})));
  MpcSignalBus bus
    "Bus connector with control variables and outputs"
    annotation (Placement(transformation(extent={{-20,80},{20,120}})));

  expandable connector MpcSignalBus  "Icon for signal bus"
    extends Modelica.Icons.SignalBus;
  Modelica.Blocks.Interfaces.RealInput u1;
  Modelica.Blocks.Interfaces.RealInput u2;
  Modelica.Blocks.Interfaces.RealInput u3;
  Modelica.Blocks.Interfaces.RealInput slack;
  Modelica.Blocks.Interfaces.RealInput Tsta;
  Modelica.Blocks.Interfaces.RealInput TGround1;
  Modelica.Blocks.Interfaces.RealInput TGround2;
  Modelica.Blocks.Interfaces.RealInput TGround3;
  Modelica.Blocks.Interfaces.RealInput TGround4;
  Modelica.Blocks.Interfaces.RealInput TGround5;
  Modelica.Blocks.Interfaces.RealInput TGround6;
  Modelica.Blocks.Interfaces.RealInput TGround7;
  Modelica.Blocks.Interfaces.RealInput TGround8;
  Modelica.Blocks.Interfaces.RealInput TGround9;
  Modelica.Blocks.Interfaces.RealInput TGround10;
  Modelica.Blocks.Interfaces.RealInput W_comp;
  end MpcSignalBus;
equation
  connect(u1, bus.u1);
  connect(u2, bus.u2);
  connect(u3, bus.u3);
  connect(slack, bus.slack);
  connect(Tsta, bus.Tsta);
  connect(TGround1, bus.TGround1);
  connect(TGround2, bus.TGround2);
  connect(TGround3, bus.TGround3);
  connect(TGround4, bus.TGround4);
  connect(TGround5, bus.TGround5);
  connect(TGround6, bus.TGround6);
  connect(TGround7, bus.TGround7);
  connect(TGround8, bus.TGround8);
  connect(TGround9, bus.TGround9);
  connect(TGround10, bus.TGround10);
  connect(W_comp, bus.W_comp);
end MpcCase900GEOTABS;
