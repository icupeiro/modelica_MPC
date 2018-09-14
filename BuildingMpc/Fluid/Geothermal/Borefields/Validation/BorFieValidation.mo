within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model BorFieValidation
  extends Modelica.Icons.Example;
  package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

  parameter Integer nSeg(min=1) = 10
    "Number of segments to use in vertical discretization of the boreholes";
  parameter Integer nHor(min=1) = 10
    "Number of cells to use in radial discretization of soil";
  parameter Modelica.SIunits.Temperature T_start = 273.15 + 10.8
    "Initial soil temperature";

  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.ExampleBorefieldData
    borFieDat(           soiDat=
      IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.SoilData.SandStone(
      steadyState=false), filDat=
        IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.FillingData.Bentonite(
         steadyState=false),
  conDat=
      IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.ConfigurationData.ExampleConfigurationData(
      nbBor=4, cooBor={{0,0},{0,6},{6,0},{6,6}}))
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  replaceable
  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.BorefieldOneUTube borFie(
  redeclare package Medium = Medium,
  borFieDat=borFieDat,
  dp_nominal=10,
  m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  T_start=273.15 + 13.5,
  TMedGro=291.15) constrainedby
    IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Boreholes.BoreholeOneUTube(
  redeclare package Medium = Medium,
  borFieDat=borFieDat,
  dp_nominal=10,
  m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  T_start=273.15 + 13.5,
    annotation (uses(Modelica(version="3.2.2"))))
  "Borehole connected to a discrete ground model" annotation (
    Placement(transformation(
      extent={{-14,-14},{14,14}},
      rotation=0,
      origin={0,0})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn(
      redeclare package Medium = Medium, m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  tau=0)                                 "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  IBPSA.Fluid.Sources.MassFlowSource_T sou(
    redeclare package Medium = Medium,
    nPorts=1,
    use_T_in=false,
  m_flow=borFieDat.conDat.mBor_flow_nominal,
  T=277.15)   "Source" annotation (Placement(transformation(extent={{-76,-10},{
            -56,10}}, rotation=0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorDisOut(
      redeclare package Medium = Medium, m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  tau=0)                                 "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  IBPSA.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    use_p_in=false,
    use_T_in=false,
    nPorts=2,
    p=101330,
    T=283.15) "Sink" annotation (Placement(transformation(extent={{90,-12},{70,
            8}},  rotation=0)));
  Development.MultipleBorehole
                 multipleBorehole(
    borFieDat=borFieDat,
    redeclare package Medium = Medium,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  dp_nominal=1000000)
    annotation (Placement(transformation(extent={{-10,-70},{10,-50}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorConOut(
      redeclare package Medium = Medium, m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  tau=0)                                 "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{30,-70},{50,-50}})));
  IBPSA.Fluid.Sources.MassFlowSource_T sou1(
  redeclare package Medium = Medium,
  nPorts=1,
  use_T_in=false,
  m_flow=borFieDat.conDat.mBor_flow_nominal,
  T=277.15)   "Source" annotation (Placement(transformation(extent={{-76,-70},
          {-56,-50}}, rotation=0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn1(
  redeclare package Medium = Medium,
  m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  tau=0)                                 "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-50,-70},{-30,-50}})));
Modelica.Blocks.Math.Add add(k2=-1)
  annotation (Placement(transformation(extent={{60,38},{80,58}})));
Modelica.Blocks.Interfaces.RealOutput error
  annotation (Placement(transformation(extent={{100,38},{120,58}})));
Modelica.Blocks.Sources.Constant TGround(k=273.15 + 10.8)
  annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
equation
  connect(sou.ports[1], TBorIn.port_a)
    annotation (Line(points={{-56,0},{-50,0}}, color={0,127,255}));
connect(TBorIn.port_b, borFie.port_a)
  annotation (Line(points={{-30,0},{-14,0}}, color={0,127,255}));
connect(borFie.port_b, TBorDisOut.port_a)
  annotation (Line(points={{14,0},{30,0}}, color={0,127,255}));
  connect(TBorDisOut.port_b, sin.ports[1])
    annotation (Line(points={{50,0},{70,0},{70,0}}, color={0,127,255}));
connect(multipleBorehole.port_b, TBorConOut.port_a)
  annotation (Line(points={{10,-60},{30,-60}}, color={0,127,255}));
  connect(TBorConOut.port_b, sin.ports[2])
    annotation (Line(points={{50,-60},{50,-4},{70,-4}}, color={0,127,255}));
connect(sou1.ports[1], TBorIn1.port_a)
  annotation (Line(points={{-56,-60},{-50,-60}}, color={0,127,255}));
connect(TBorIn1.port_b, multipleBorehole.port_a)
  annotation (Line(points={{-30,-60},{-10,-60}}, color={0,127,255}));
connect(add.y, error)
  annotation (Line(points={{81,48},{110,48}}, color={0,0,127}));
connect(TBorDisOut.T, add.u1) annotation (Line(points={{40,11},{40,54},
        {58,54}}, color={0,0,127}));
connect(TBorConOut.T, add.u2) annotation (Line(points={{40,-49},{48,
        -49},{48,42},{58,42}}, color={0,0,127}));
connect(TGround.y, borFie.TSoi) annotation (Line(points={{-59,50},{
        -30,50},{-30,8.4},{-16.8,8.4}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
  experiment(
    StopTime=86400,
    Tolerance=1e-06,
    __Dymola_fixedstepsize=10,
    __Dymola_Algorithm="Euler"),
  __Dymola_experimentSetupOutput,
  __Dymola_experimentFlags(
    Advanced(
      GenerateVariableDependencies=false,
      OutputModelicaCode=false,
      InlineMethod=0,
      InlineOrder=2,
      InlineFixedStep=0.001),
    Evaluate=false,
    OutputCPUtime=false,
    OutputFlatModelica=false));
end BorFieValidation;
