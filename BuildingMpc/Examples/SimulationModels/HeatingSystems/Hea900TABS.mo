within BuildingMpc.Examples.SimulationModels.HeatingSystems;
model Hea900TABS
  extends IDEAS.Templates.Interfaces.BaseClasses.HeatingSystem(
  nZones=1,
  nLoads=1,
  nEmbPorts=1,
  nTemSen=1,
  isCoo=true,
  nConvPorts=1,
  nRadPorts=1,
  P={0},
  Q={0},
  Q_design={3000},
  QHeaSys=max(0,heatPortEmb[1].Q_flow),
  QCooTotal=min(0,heatPortEmb[1].Q_flow));
  MPCs.MpcCase900TABS mpcCase900TABS(stateEstimationType=UnitTests.MPC.BaseClasses.StateEstimationType.Perfect)
    annotation (Placement(transformation(extent={{-102,54},{-82,74}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=mpcCase900TABS.yOpt[1])
    annotation (Placement(transformation(extent={{-130,50},{-150,70}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow
    annotation (Placement(transformation(extent={{-160,50},{-180,70}})));
  Modelica.Blocks.Interfaces.RealInput[mpcCase900TABS.nSta] u annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-122,100})));
equation
     heatPortCon.Q_flow = {0};
   heatPortRad.Q_flow = {0};
  connect(prescribedHeatFlow.port, heatPortEmb[1])
    annotation (Line(points={{-180,60},{-200,60}}, color={191,0,0}));
  connect(realExpression.y, prescribedHeatFlow.Q_flow)
    annotation (Line(points={{-151,60},{-160,60}}, color={0,0,127}));
  connect(u, mpcCase900TABS.uSta) annotation (Line(points={{-122,100},{-122,83},
          {-104,83},{-104,66}}, color={0,0,127}));
end Hea900TABS;
