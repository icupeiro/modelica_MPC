within BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses;
model CylindricalGroundLayer
  "Heat conduction in a cylinder using the radial discretization as advised by Eskilson"
  parameter Integer nSta(min=4) = 10 "number of discretisations";
  parameter
    IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.SoilData.Template soiDat
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  parameter Modelica.SIunits.Height h "Height of the cylinder";
  parameter Modelica.SIunits.Radius r_a "Internal radius";
  parameter Modelica.SIunits.Radius r_b "External radius";
  parameter Modelica.SIunits.Temperature TInt_start=293.15
    "Initial temperature at port_a, used if steadyStateInitial = false"
    annotation (Dialog(group="Initialization", enable=not steadyStateInitial));
  parameter Modelica.SIunits.Temperature TExt_start=293.15
    "Initial temperature at port_b, used if steadyStateInitial = false"
    annotation (Dialog(group="Initialization", enable=not steadyStateInitial));
  parameter Boolean steadyStateInitial=true
    "true initializes dT(0)/dt=0, false initializes T(0) at fixed temperature using T_a_start and T_b_start"
    annotation (Dialog(group="Initialization"), Evaluate=true);

  parameter Real gridFac(min=1) = 2 "Grid factor for spacing";

  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a(T(start=TInt_start))
    "Heat port at surface a" annotation (Placement(transformation(extent={{-110,
            -10},{-90,10}}, rotation=0)));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b(T(start=TExt_start))
    "Heat port at surface b" annotation (Placement(transformation(extent={{90,-10},
            {110,10}},rotation=0)));

  parameter Modelica.SIunits.Radius r[nSta + 1](each fixed=false)
    "Radius to the boundary of the i-th domain";
  parameter Real beta[nSta] = 2*ones(nSta);
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor[nSta] Rsoil(R = {1/G[i] for i in 1:nSta})
    "Borehole resistance" annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=180,
        origin={0,0})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor[nSta] Csoil(C=C, der_T(
        fixed=true))
    annotation (Placement(transformation(extent={{-12,42},{12,66}})));

protected
  parameter Modelica.SIunits.Radius rC[nSta] = {(r[i] + r[i + 1])/2 for i in 1:nSta}
    "Radius to the center of the i-th domain";

  final parameter Modelica.SIunits.SpecificHeatCapacity c=soiDat.c
    "Specific heat capacity";
  final parameter Modelica.SIunits.ThermalConductivity k=soiDat.k
    "Thermal conductivity of the material";
  final parameter Modelica.SIunits.Density d=soiDat.d
    "Density of the material";

  parameter Modelica.SIunits.ThermalConductance G[nSta] = cat(1, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(rC[1]/r_a)}, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(rC[i]/rC[i - 1]) for i in 2:nSta-1}, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(r_b/rC[nSta])})
    "Heat conductance between the temperature nodes";
  parameter Modelica.SIunits.HeatCapacity C[nSta] = {d*Modelica.Constants.pi*c*h*((r[i+1]^2 - (r[i])^2)) for i in 1:nSta}
        "Heat capacity of each state";
  parameter Real gridFac_sum(fixed=false);
  parameter Real gridFac_sum_old(fixed=false);

initial algorithm
  for i in 0:nSta - 3 - 1 loop
    if i == 0 then
      gridFac_sum := gridFac^i;
      gridFac_sum_old := gridFac_sum;
    else
      gridFac_sum := gridFac_sum_old + gridFac^i;
      gridFac_sum_old := gridFac_sum;
    end if;
  end for;
initial equation
  r[1] = r_a;
  r[2] = r_a + sqrt(k/c/d*60) "eskilson minimum length";
  r[3] = r_a + 2*sqrt(k/c/d*60);
  r[4] = r_a + 3*sqrt(k/c/d*60);

  for i in 5:nSta + 1 loop
    r[i] = r[i - 1] + (r_b - r[4])/gridFac_sum*gridFac^(i - 5);
  end for;

equation

  connect(port_a, Rsoil[1].port_a)
    annotation (Line(points={{-100,0},{-12,0}}, color={191,0,0}));
  connect(Rsoil[nSta].port_a, Csoil[nSta].port)  annotation (Line(points={{-12,0},
          {-10,0},{-10,32},{0,32},{0,42}},
                                      color={191,0,0}));
  connect(Rsoil[nSta].port_b, port_b)
    annotation (Line(points={{12,0},{100,0}}, color={191,0,0}));
  connect(Rsoil[nSta].port_b, Csoil[nSta].port);
  for i in 1:(nSta-1) loop
    connect(Rsoil[i].port_a, Csoil[i].port);
    connect(Rsoil[i].port_b, Rsoil[i+1].port_a) annotation (Line(points={{12,0},{28,0},{28,
          -22},{-24,-22},{-24,0},{-12,0},{-12,1.44329e-15}}, color={191,0,0}));
  end for;






  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
            100,100}})),
    Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
                    graphics={
        Rectangle(
          extent={{-94,4},{92,-4}},
          lineColor={0,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{12,8},{14,8},{16,4},{18,-2},{18,-6},{16,-12},{10,-16},{6,-20},
              {-2,-22},{-6,-18},{-12,-12},{-14,-2},{-12,4},{-10,8},{-8,10},{-6,
              12},{-2,14},{2,14},{8,12},{12,8}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={215,215,215},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-6,-16},{2,-18},{8,-16},{14,-14},{10,-16},{6,-20},{-2,-22},{
              -8,-20},{-12,-12},{-14,-2},{-12,4},{-10,8},{-8,10},{-10,0},{-10,-8},
              {-6,-16}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-110,-74},{-26,-86}},
          lineColor={0,0,255},
          textString="%r_a"),
        Text(
          extent={{-22,-62},{20,-76}},
          lineColor={0,0,255},
          textString="%nSta"),
        Text(
          extent={{16,-76},{102,-88}},
          lineColor={0,0,255},
          textString="%r_b"),
        Polygon(
          points={{-50,60},{-38,34},{-32,0},{-36,-30},{-50,-60},{-62,-60},{-48,
              -30},{-44,0},{-50,34},{-62,60},{-50,60}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillPattern=FillPattern.Backward,
          fillColor={175,175,175}),
        Polygon(
          points={{52,60},{64,34},{70,0},{66,-30},{52,-60},{40,-60},{54,-30},{
              58,0},{52,34},{40,60},{52,60}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillPattern=FillPattern.Backward,
          fillColor={175,175,175}),
        Text(
          extent={{-100,100},{100,60}},
          lineColor={0,0,255},
          textString="%name")}),
    defaultComponentName="lay",
    Documentation(info="<html>
<p>
Model for radial heat transfer in a hollow cylinder.
</p>
<p>
If the heat capacity of the material is non-zero, then this model computes transient heat conduction, i.e., it
computes a numerical approximation to the solution of the heat equation
</p>
<p align=\"center\" style=\"font-style:italic;\">
   &rho; c ( &part; T(r,t) &frasl; &part;t ) = 
    k ( &part;&sup2; T(r,t) &frasl; &part;r&sup2; + 1 &frasl; r &nbsp;  &part; T(r,t) &frasl; &part;r ),
</p>
<p>
where 
<i>&rho;</i>
is the mass density,
<i>c</i>
is the specific heat capacity per unit mass,
<i>T</i>
is the temperature at location <i>r</i> and time <i>t</i> and
<i>k</i> is the heat conductivity. 
At the locations <i>r=r<sub>a</sub></i> and <i>r=r<sub>b</sub></i>, 
the temperature and heat flow rate are equal to the 
temperature and heat flow rate of the heat ports.
</p>
<p>
If the heat capacity of the material is set to zero, then steady-state heat flow is computed using
</p>
<p align=\"center\" style=\"font-style:italic;\">
   Q = 2 &pi; k (T<sub>a</sub>-T<sub>b</sub>)&frasl; ln(r<sub>a</sub> &frasl; r<sub>b</sub>),
</p>
<p>
where
<i>r<sub>a</sub></i> is the internal radius,
<i>r<sub>b</sub></i> is the external radius,
<i>T<sub>a</sub></i> is the temperature at port a and
<i>T<sub>b</sub></i> is the temperature at port b.
</p>
<h4>Implementation</h4>
<p>
To spatially discretize the heat equation, the construction is 
divided into compartments with <code>material.nSta &ge; 1</code> state variables. 
The state variables are connected to each other through thermal conductors. 
There is also a thermal conductor
between the surfaces and the outermost state variables. Thus, to obtain
the surface temperature, use <code>port_a.T</code> (or <code>port_b.T</code>)
and not the variable <code>T[1]</code>.
</p>
</html>", revisions="<html>
<ul>
<li>
Januari, 2014, by Damien Picard:<br/>
Modify the discretization of the cilindrical layer so that the first three layers have an equal thickness the following an exponentionally growing thickness.
This follows the guidelines of Eskilson (P. Eskilson. Thermal analysis of heat extraction
boreholes. PhD thesis, Dep. of Mathematical
Physics, University of Lund, Sweden, 1987).
</li>
<li>
March 9, 2012, by Michael Wetter:<br/>
Removed protected variable <code>der_T</code> as it is not required.
</li>
<li>
April 14 2011, by Pierre Vigouroux:<br/>
First implementation.
</li>
</ul>
</html>"));
end CylindricalGroundLayer;
