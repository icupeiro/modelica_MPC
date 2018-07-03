within BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers;
model SingleBorehole "single borehole model for MPC"

  replaceable package Medium =
      Modelica.Media.Interfaces.PartialMedium "Medium through the borehole"
      annotation (choicesAllMatching = true);

  BaseClasses.SingleBoreHoleUTube
    borehole(
    allowFlowReversal=false,
    computeFlowResistance=false,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    intHex(
    Q2_flow(       nominal = -65*borehole.hSeg),
    Q1_flow( nominal = 65*borehole.hSeg)),
    borFieDat=borFieDat,
  energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    nSeg=10)                            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,0})));
  BaseClasses.CylindricalGroundLayer
    lay[nSeg](
    each r_a=borFieDat.conDat.rBor,
    each r_b=3,
    each soiDat=borFieDat.soiDat,
  steadyStateInitial=false,
    each nSta=nHor,
    each h=borFieDat.conDat.hBor/nSeg,
    each TInt_start=T_start,
    each TExt_start=T_start)                        annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40})));

  parameter Integer nSeg(min=1) = 10
    "Number of segments to use in vertical discretization of the boreholes";
  parameter Integer nHor(min=1) = 10
    "Number of cells to use in radial discretization of soil";
  parameter Modelica.SIunits.Temperature T_start = 273.15 + 10
    "Initial soil temperature";

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
    borFieDat(
    filDat=borFieDat.filDat,
    soiDat=borFieDat.soiDat,
    conDat=borFieDat.conDat)=
                borFieDat "Borefield parameters";
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

public
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor[nSeg] Cground(der_T(
        fixed=true),
  C=Modelica.Constants.inf,
  T(start=soilTemp, fixed=true))
    annotation (Placement(transformation(extent={{-12,72},{12,96}})));
equation
  connect(lay.port_a, borehole.port_wall) annotation (Line(points={{-4.44089e-16,
          30},{0,30},{0,10}}, color={191,0,0}));
  connect(port_a, borehole.port_a)
    annotation (Line(points={{-100,0},{-10,0}},         color={0,127,255}));
  connect(borehole.port_b, port_b)
    annotation (Line(points={{10,0},{100,0}},         color={0,127,255}));
connect(Cground.port, lay.port_b)
  annotation (Line(points={{0,72},{0,50}}, color={191,0,0}));
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
end SingleBorehole;
