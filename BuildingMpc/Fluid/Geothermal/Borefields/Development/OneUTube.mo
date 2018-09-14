within BuildingMpc.Fluid.Geothermal.Borefields.Development;
model OneUTube
  "controller model for single-U configuration borefield model"

    replaceable package Medium =
      Modelica.Media.Interfaces.PartialMedium "Medium through the borehole"
      annotation (choicesAllMatching = true);

    parameter Modelica.SIunits.Temperature Tsoil=T(fixed=true)
    "Undisturbed temperature of the ground";
    parameter Integer nSeg = 10
     "Number of segments to use in vertical discretization of the boreholes";

  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    p(start=Medium.p_default),
    redeclare final package Medium = Medium,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0),
    h_outflow(start=Medium.h_default, nominal=Medium.h_default))
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    p(start=Medium.p_default),
    redeclare final package Medium = Medium,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0),
    h_outflow(start=Medium.h_default, nominal=Medium.h_default))
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{110,-10},{90,10}})));
  IBPSA.Fluid.BaseClasses.MassFlowRateMultiplier masFloDiv(
    redeclare final package Medium = Medium,
    final allowFlowReversal=allowFlowReversal,
    final k=borFieDat.conDat.nBor) "Division of flow rate"
    annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));
              IBPSA.Fluid.Geothermal.Borefields.BaseClasses.Boreholes.OneUTube                    borHol(
    redeclare final package Medium = Medium,
    final borFieDat=borFieDat,
    final nSeg=nSeg,
    final allowFlowReversal=allowFlowReversal,
    final m_flow_small=m_flow_small,
    final show_T=show_T,
    final computeFlowResistance=computeFlowResistance,
    final from_dp=from_dp,
    final linearizeFlowResistance=linearizeFlowResistance,
    final deltaM=deltaM,
    final TGro_start=TGro_start,
    final TFlu_start=TFlu_start,
    final energyDynamics=energyDynamics,
    final p_start=p_start,
    final dynFil=dynFil,
    final m_flow_nominal=m_flow_nominal,
    final dp_nominal=dp_nominal,
    intHex(RVol1(y=
            IBPSA.Fluid.Geothermal.Borefields.BaseClasses.Boreholes.BaseClasses.Functions.convectionResistanceCircularPipe(
            hSeg=borFieDat.conDat.hBor/nSeg,
            rTub=borFieDat.conDat.rTub,
            eTub=borFieDat.conDat.eTub,
            kMed=kMed,
            muMed=muMed,
            cpMed=cpMed,
            m_flow=borFieDat.conDat.mBor_flow_nominal,
            m_flow_nominal=borFieDat.conDat.mBor_flow_nominal)), RVol2(y=
            IBPSA.Fluid.Geothermal.Borefields.BaseClasses.Boreholes.BaseClasses.Functions.convectionResistanceCircularPipe(
            hSeg=borFieDat.conDat.hBor/nSeg,
            rTub=borFieDat.conDat.rTub,
            eTub=borFieDat.conDat.eTub,
            kMed=kMed,
            muMed=muMed,
            cpMed=cpMed,
            m_flow=borFieDat.conDat.mBor_flow_nominal,
            m_flow_nominal=borFieDat.conDat.mBor_flow_nominal))))
                     "Borehole"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  IBPSA.Fluid.BaseClasses.MassFlowRateMultiplier masFloMul(
    redeclare final package Medium = Medium,
    final allowFlowReversal=allowFlowReversal,
    final k=borFieDat.conDat.nBor) "Mass flow multiplier"
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  BaseClasses.CylindricalGroundLayer
    lay[nSeg](
    each r_b=3,
    each soiDat=borFieDat.soiDat,
    steadyStateInitial=false,
    each h=borFieDat.conDat.hBor/nSeg,
    each r_a=borFieDat.conDat.rBor,
    TInt_start=TGro_start,
    TExt_start=Tsoil)                               annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor[nSeg] Cground(
    C=Modelica.Constants.inf,
    T(start=Tsoil, fixed=true),
    der_T(fixed=true))
    annotation (Placement(transformation(extent={{-12,72},{12,96}})));
  parameter Boolean allowFlowReversal=true
    "= false to simplify equations, assuming, but not enforcing, no flow reversal"
    annotation (Dialog(tab="Assumptions"));
  parameter Modelica.SIunits.MassFlowRate m_flow_small=1E-4*abs(borHol.m_flow_nominal)
    "Small mass flow rate for regularization of zero flow"
    annotation (Dialog(tab="Advanced"));
  parameter Boolean show_T=false
    "= true, if actual temperature at port is computed"
    annotation (Dialog(tab="Advanced", group="Diagnostics"));
  parameter Boolean computeFlowResistance=borHol.dp_nominal > Modelica.Constants.eps
    "=true, compute flow resistance. Set to false to assume no friction"
    annotation (Dialog(tab="Flow resistance"));
  parameter Boolean from_dp=false
    "= true, use m_flow = f(dp) else dp = f(m_flow)"
    annotation (Dialog(tab="Flow resistance"));
  parameter Boolean linearizeFlowResistance=false
    "= true, use linear relation between m_flow and dp for any flow rate"
    annotation (Dialog(tab="Flow resistance"));
  parameter Real deltaM=0.1
    "Fraction of nominal flow rate where flow transitions to laminar"
    annotation (Dialog(tab="Flow resistance"));
  parameter Modelica.SIunits.Temperature TGro_start[nSeg]
    "Start value of grout temperature" annotation (Dialog(tab="Initialization"));
  parameter Modelica.SIunits.Temperature TFlu_start[nSeg]=borHol.TGro_start
    "Start value of fluid temperature" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Media.Interfaces.Types.AbsolutePressure p_start=Medium.p_default
    "Start value of pressure" annotation (Dialog(tab="Initialization"));
  parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
    "Type of energy balance: dynamic (3 initialization options) or steady state"
    annotation (Dialog(tab="Dynamics"));
  parameter Boolean dynFil=true
    "Set to false to remove the dynamics of the filling material"
    annotation (Dialog(tab="Dynamics"));
  parameter IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Template borFieDat "Borefield data"
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal
    "Nominal mass flow rate";
  parameter Modelica.SIunits.PressureDifference dp_nominal
    "Pressure difference";

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
  parameter Modelica.SIunits.DynamicViscosity muMed=Medium.dynamicViscosity(
      Medium.setState_pTX(
      Medium.p_default,
      Medium.T_default,
      Medium.X_default)) "Dynamic viscosity of the fluid";

equation
  connect(port_a, masFloDiv.port_b)
    annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
  connect(masFloDiv.port_a, borHol.port_a)
    annotation (Line(points={{-60,0},{-10,0}}, color={0,127,255}));
  connect(borHol.port_b, masFloMul.port_a)
    annotation (Line(points={{10,0},{60,0}}, color={0,127,255}));
  connect(masFloMul.port_b, port_b)
    annotation (Line(points={{80,0},{100,0}}, color={0,127,255}));
connect(Cground.port,lay. port_b)
  annotation (Line(points={{0,72},{0,50}}, color={191,0,0}));
  connect(lay.port_a, borHol.port_wall)
    annotation (Line(points={{0,30},{0,10}}, color={191,0,0}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,60},{100,-66}},
          lineColor={0,0,0},
          fillColor={234,210,210},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-88,-6},{-32,-62}},
          lineColor={0,0,0},
          fillColor={223,188,190},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{-82,-12},{-38,-56}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{-88,54},{-32,-2}},
          lineColor={0,0,0},
          fillColor={223,188,190},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{-82,48},{-38,4}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{-26,54},{30,-2}},
          lineColor={0,0,0},
          fillColor={223,188,190},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{-20,48},{24,4}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{-28,-6},{28,-62}},
          lineColor={0,0,0},
          fillColor={223,188,190},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{-22,-12},{22,-56}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{36,56},{92,0}},
          lineColor={0,0,0},
          fillColor={223,188,190},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{42,50},{86,6}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{38,-4},{94,-60}},
          lineColor={0,0,0},
          fillColor={223,188,190},
          fillPattern=FillPattern.Forward),
        Ellipse(
          extent={{44,-10},{88,-54}},
          lineColor={0,0,0},
          fillColor={0,0,255},
          fillPattern=FillPattern.Forward)}), Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end OneUTube;
