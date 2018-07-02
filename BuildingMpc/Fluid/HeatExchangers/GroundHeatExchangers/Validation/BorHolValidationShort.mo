within BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers.Validation;
model BorHolValidationShort
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
         steadyState=false))
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  Modelica.Blocks.Sources.Constant TGroUn(k=273.15 + 10.8)
    "Undisturbed ground temperature" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,90})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature preTem
    "Prescribed temperature" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-50,70})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector therCol(m=nSeg)
    "Thermal collector" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,70})));
  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.GroundHeatTransfer.CylindricalGroundLayer
    lay[nSeg](
    each soiDat=borFieDat.soiDat,
    each r_a=borFieDat.conDat.rBor,
    each r_b=3,
    each h=borFieDat.conDat.hBor/nSeg,
    each nSta=nHor,
    each TInt_start=T_start,
    each TExt_start=T_start)                  annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40})));
  replaceable
    IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Boreholes.BoreholeOneUTube
    borHolDis(
    redeclare package Medium = Medium,
    borFieDat=borFieDat,
    dp_nominal=10,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  T_start=273.15 + 13.5)
  "Borehole connected to a discrete ground model"                  annotation (
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
  SingleBorehole singleBorehole(
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
equation
  connect(TGroUn.y, preTem.T)
    annotation (Line(points={{-80,79},{-80,70},{-62,70}}, color={0,0,127}));
  connect(preTem.port, therCol.port_b)
    annotation (Line(points={{-40,70},{-30,70}}, color={191,0,0}));
  connect(therCol.port_a, lay.port_b)
    annotation (Line(points={{-10,70},{0,70},{0,50}}, color={191,0,0}));
  connect(lay.port_a, borHolDis.port_wall)
    annotation (Line(points={{0,30},{0,14}}, color={191,0,0}));
  connect(sou.ports[1], TBorIn.port_a)
    annotation (Line(points={{-56,0},{-50,0}}, color={0,127,255}));
  connect(TBorIn.port_b, borHolDis.port_a)
    annotation (Line(points={{-30,0},{-14,0}}, color={0,127,255}));
  connect(borHolDis.port_b, TBorDisOut.port_a)
    annotation (Line(points={{14,0},{30,0}}, color={0,127,255}));
  connect(TBorDisOut.port_b, sin.ports[1])
    annotation (Line(points={{50,0},{70,0},{70,0}}, color={0,127,255}));
  connect(singleBorehole.port_b, TBorConOut.port_a)
    annotation (Line(points={{10,-60},{30,-60}}, color={0,127,255}));
  connect(TBorConOut.port_b, sin.ports[2])
    annotation (Line(points={{50,-60},{50,-4},{70,-4}}, color={0,127,255}));
connect(sou1.ports[1], TBorIn1.port_a)
  annotation (Line(points={{-56,-60},{-50,-60}}, color={0,127,255}));
connect(TBorIn1.port_b, singleBorehole.port_a)
  annotation (Line(points={{-30,-60},{-10,-60}}, color={0,127,255}));
connect(add.y, error)
  annotation (Line(points={{81,48},{110,48}}, color={0,0,127}));
connect(TBorDisOut.T, add.u1) annotation (Line(points={{40,11},{40,54},
        {58,54}}, color={0,0,127}));
connect(TBorConOut.T, add.u2) annotation (Line(points={{40,-49},{48,
        -49},{48,42},{58,42}}, color={0,0,127}));
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
end BorHolValidationShort;
