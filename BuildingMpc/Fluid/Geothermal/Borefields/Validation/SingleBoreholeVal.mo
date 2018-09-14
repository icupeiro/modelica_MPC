within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model SingleBoreholeVal "single borehole model for MPC"

  replaceable package Medium =
      Modelica.Media.Interfaces.PartialMedium "Medium through the borehole"
      annotation (choicesAllMatching = true);

  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Boreholes.BoreholeOneUTube
    borehole(
    allowFlowReversal=false,
    computeFlowResistance=false,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    intHex(
    Q2_flow(       nominal = -65*borFieDat.conDat.hSeg),
    Q1_flow( nominal = 65*borFieDat.conDat.hSeg)),
    borFieDat=borFieDat,
  energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  T_start=285.55)                       annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,0})));
  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.GroundHeatTransfer.CylindricalGroundLayer
    lay[borFieDat.conDat.nVer](
    each h=borFieDat.conDat.hSeg,
    each r_a=borFieDat.conDat.rBor,
    each r_b=3,
    each nSta=borFieDat.conDat.nHor,
    each TInt_start=borFieDat.conDat.T_start,
    each TExt_start=borFieDat.conDat.T_start,
    each soiDat=borFieDat.soiDat,
    steadyStateInitial=true)                        annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40})));
  parameter Modelica.SIunits.Temperature soilTemp=273.15 + 10.8
    "Undisturbed temperature of the ground";
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal
    "Nominal mass flow rate through the borehole"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.PressureDifference dp_nominal
    "Pressure difference through the borehole"
    annotation (Dialog(group="Nominal condition"));
  parameter
    IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.Template
    borFieDat "Borefield parameters";
Modelica.Blocks.Interfaces.RealInput TGround annotation (Placement(
      transformation(
      extent={{-20,-20},{20,20}},
      rotation=-90,
      origin={0,120})));
Modelica.Blocks.Routing.Replicator replicator(nout=borFieDat.conDat.nVer)
  annotation (Placement(transformation(
      extent={{-4,-4},{4,4}},
      rotation=-90,
      origin={4.44089e-16,90})));
protected
    parameter Modelica.SIunits.SpecificHeatCapacity cpMed=
      Medium.specificHeatCapacityCp(Medium.setState_pTX(
      Medium.p_default,
      Medium.T_default,
      Medium.X_default)) "Specific heat capacity of the fluid";
  parameter Modelica.SIunits.ThermalConductivity kMed=
      Medium.thermalConductivity(Medium.setState_pTX(
      Medium.p_default,
      Medium.T_default,
      Medium.X_default)) "Thermal conductivity of the fluid";
  parameter Modelica.SIunits.DynamicViscosity mueMed=Medium.dynamicViscosity(
      Medium.setState_pTX(
      Medium.p_default,
      Medium.T_default,
      Medium.X_default)) "Dynamic viscosity of the fluid";
 Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature[borFieDat.conDat.nVer]
    prescribedTemperature(T(start=273.15 + 10.8))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
      rotation=-90,
      origin={0,70})));
equation
  connect(lay.port_a, borehole.port_wall) annotation (Line(points={{-4.44089e-16,
          30},{0,30},{0,10}}, color={191,0,0}));
  connect(port_a, borehole.port_a)
    annotation (Line(points={{-100,0},{-10,0}},         color={0,127,255}));
  connect(borehole.port_b, port_b)
    annotation (Line(points={{10,0},{100,0}},         color={0,127,255}));
connect(prescribedTemperature.port, lay.port_b)
  annotation (Line(points={{0,60},{0,50}}, color={191,0,0}));
connect(prescribedTemperature.T, prescribedTemperature.T)
  annotation (Line(points={{0,82},{0,82}}, color={0,0,127}));
connect(replicator.u, TGround)
  annotation (Line(points={{0,94.8},{0,120}}, color={0,0,127}));
connect(prescribedTemperature.T, replicator.y)
  annotation (Line(points={{0,82},{0,85.6}}, color={0,0,127}));
                 annotation (Placement(transformation(extent={{-10,-10},{10,10}})),
              Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-70,80},{70,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,-52},{62,-60}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,58},{62,54}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,6},{62,0}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{54,92},{46,-88}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-54,-88},{-46,92}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-72,80},{-62,-80}},
          lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{62,80},{72,-80}},
          lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-52,-80},{54,-88}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SingleBoreholeVal;
