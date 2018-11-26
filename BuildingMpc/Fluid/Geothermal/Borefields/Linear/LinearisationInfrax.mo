within BuildingMpc.Fluid.Geothermal.Borefields.Linear;
model LinearisationInfrax
  extends Modelica.Icons.Example;
  package Medium = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=283.15, X_a=0.30);
      parameter Modelica.SIunits.TemperatureDifference dT = 3 "design temperature difference in the borefield";
  TwoUTube twoUTube(
    borFieDat=iNFRAX_bF,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    TGro_start=(273.15 + 13.5)*ones(10),
    show_T=true,
    TExt0_start=273.15 + 13.5,
    z0=0,
    dT_dz=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dynFil=true)
    annotation (Placement(transformation(extent={{-30,-20},{10,20}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(
    redeclare package Medium = Medium,
    tau=0,
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut(
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  INFRAX.Data.Parameters.BorefieldData.INFRAX_bF iNFRAX_bF
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  IDEAS.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    use_T_in=true,
    nPorts=1)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  IDEAS.Fluid.Sources.Boundary_pT sink(redeclare package Medium = Medium,
      nPorts=1)
    annotation (Placement(transformation(extent={{100,-10},{80,10}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{50,-10},{70,10}})));
public
  Modelica.Blocks.Sources.RealExpression realExpression1(y=TOut.T)
    annotation (Placement(transformation(extent={{20,40},{0,60}})));
  Modelica.Blocks.Sources.Constant const(k=5)
    annotation (Placement(transformation(extent={{0,70},{20,90}})));
  Buses.InputBus inputBus annotation (Placement(transformation(extent={{-120,58},
            {-80,98}}), iconTransformation(extent={{-208,28},{-188,48}})));
  Buses.OutputBus outputBus
    annotation (Placement(transformation(extent={{-120,30},{-80,70}})));
equation
  connect(TIn.port_b,twoUTube. port_a)
    annotation (Line(points={{-40,0},{-30,0}}, color={0,127,255}));
  connect(twoUTube.port_b,TOut. port_a)
    annotation (Line(points={{10,0},{20,0}}, color={0,127,255}));
  connect(TOut.port_b,mFlow. port_a)
    annotation (Line(points={{40,0},{50,0}}, color={0,127,255}));
  connect(mFlow.port_b,sink. ports[1])
    annotation (Line(points={{70,0},{80,0}}, color={0,127,255}));
  connect(sou.ports[1],TIn. port_a)
    annotation (Line(points={{-70,0},{-60,0}}, color={0,127,255}));
  connect(realExpression1.y,outputBus. TBorOut) annotation (Line(points={{-1,50},
          {-50,50},{-50,50.1},{-99.9,50.1}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(sou.T_in, inputBus.Tin) annotation (Line(points={{-92,4},{-100,4},{-100,
          74}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(const.y, mFlow.m_flow_in)
    annotation (Line(points={{21,80},{54,80},{54,8}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end LinearisationInfrax;
