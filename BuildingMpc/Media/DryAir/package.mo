within BuildingMpc.Media;
package DryAir "Package with dry air model"
  extends BuildingMpc.Media.PartialSimpleMedium(
  mediumName="Air",
  cp_const=1013,
  cv_const=768,
  d_const=1.2,
  eta_const=1,
  lambda_const=1,
  a_const=1,
  T_min=240,
  T_max=400,
  T0=273.15 + 20,
  MM_const=0.0289651159,
  reference_X={1},
  reference_T=273.15,
  reference_p=101325,
  AbsolutePressure(start=p_default),
  Temperature(start=T_default),
  extraPropertiesNames=fill("CO2", if useCO2 then 1 else 0));
  constant Boolean useCO2 = false;

  extends Modelica.Icons.Package;

  annotation(preferredView="info", Documentation(info="<html>
<p>
Very simple air medium that does not cause linear algebraic loops in JModelica.
</p>
</html>", revisions="<html>
<ul>
<li>
November 4, 2016, by Michael Wetter:<br/>
Set default value for <code>dT.start</code> in base properties.<br/>
This is for
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/575\">#575</a>.
</li>
<li>
June 6, 2015, by Michael Wetter:<br/>
Set <code>AbsolutePressure(start=p_default)</code> to avoid
a translation error if
<a href=\"modelica://Solarwind.Fluid.Sources.Examples.TraceSubstancesFlowSource\">
Solarwind.Fluid.Sources.Examples.TraceSubstancesFlowSource</a>
is translated in pedantic mode in Dymola 2016.
The reason is that pressures use <code>Medium.p_default</code> as start values,
but
<a href=\"modelica://Modelica.Media.Interfaces.Types\">
Modelica.Media.Interfaces.Types</a>
sets a default value of <i>1E-5</i>.
A similar change has been done for pressure.
This fixes
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/266\">#266</a>.
</li>
<li>
June 5, 2015, by Michael Wetter:<br/>
Added <code>stateSelect</code> attribute in <code>BaseProperties.T</code>
to allow correct use of <code>preferredMediumState</code> as
described in
<a href=\"modelica://Modelica.Media.Interfaces.PartialMedium\">
Modelica.Media.Interfaces.PartialMedium</a>.
Note that the default is <code>preferredMediumState=false</code>
and hence the same states are used as were used before.
This is for
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/260\">#260</a>.
</li>
<li>
May 11, 2015, by Michael Wetter:<br/>
Removed
<code>p(stateSelect=if preferredMediumStates then StateSelect.prefer else StateSelect.default)</code>
in declaration of <code>BaseProperties</code>.
Otherwise, when models that contain a fluid volume
are exported as an FMU, their pressure would be
differentiated with respect to time. This would require
the time derivative of the inlet pressure, which is not available,
causing the translation to stop with an error.
</li>
<li>
May 1, 2015, by Michael Wetter:<br/>
Added <code>Inline=true</code> for
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/227\">
issue 227</a>.
</li>
<li>
March 20, 2015, by Michael Wetter:<br/>
Added missing term <code>state.p/reference_p</code> in function
<code>specificEntropy</code>.
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/193\">#193</a>.
</li>
<li>
February 3, 2015, by Michael Wetter:<br/>
Removed <code>stateSelect.prefer</code> for temperature.
This is for
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/160\">#160</a>.
</li>
<li>
July 24, 2014, by Michael Wetter:<br/>
Changed implementation to use
<a href=\"modelica://Solarwind.Utilities.Psychrometrics.Constants\">
Solarwind.Utilities.Psychrometrics.Constants</a>.
This was done to use consistent values throughout the library.
</li>
<li>
November 16, 2013, by Michael Wetter:<br/>
Revised and simplified the implementation.
</li>
<li>
November 14, 2013, by Michael Wetter:<br/>
Removed function
<code>HeatCapacityOfWater</code>
which is neither needed nor implemented in the
Modelica Standard Library.
</li>
<li>
November 13, 2013, by Michael Wetter:<br/>
Removed non-used computations in <code>specificEnthalpy_pTX</code> and
in <code>temperature_phX</code>.
</li>
<li>
March 29, 2013, by Michael Wetter:<br/>
Added <code>final standardOrderComponents=true</code> in the
<code>BaseProperties</code> declaration. This avoids an error
when models are checked in Dymola 2014 in the pedenatic mode.
</li>
<li>
April 12, 2012, by Michael Wetter:<br/>
Added keyword <code>each</code> to <code>Xi(stateSelect=...</code>.
</li>
<li>
April 4, 2012, by Michael Wetter:<br/>
Added redeclaration of <code>ThermodynamicState</code> to avoid a warning
during model check and translation.
</li>
<li>
August 3, 2011, by Michael Wetter:<br/>
Fixed bug in <code>u=h-R*T</code>, which is only valid for ideal gases.
For this medium, the function is <code>u=h-pStd/dStp</code>.
</li>
<li>
January 27, 2010, by Michael Wetter:<br/>
Fixed bug in <code>else</code> branch of function <code>setState_phX</code>
that lead to a run-time error when the constructor of this function was called.
</li>
<li>
January 22, 2010, by Michael Wetter:<br/>
Added implementation of function
<a href=\"modelica://Solarwind.Media.GasesPTDecoupled.MoistAirUnsaturated.enthalpyOfNonCondensingGas\">
enthalpyOfNonCondensingGas</a> and its derivative.
<li>
January 13, 2010, by Michael Wetter:<br/>
Fixed implementation of derivative functions.
</li>
<li>
August 28, 2008, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"),
    Icon(graphics={
        Ellipse(
          extent={{-78,78},{-34,34}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={120,120,120}),
        Ellipse(
          extent={{-18,86},{26,42}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={120,120,120}),
        Ellipse(
          extent={{48,58},{92,14}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={120,120,120}),
        Ellipse(
          extent={{-22,32},{22,-12}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={120,120,120}),
        Ellipse(
          extent={{36,-32},{80,-76}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={120,120,120}),
        Ellipse(
          extent={{-36,-30},{8,-74}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={120,120,120}),
        Ellipse(
          extent={{-90,-6},{-46,-50}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={120,120,120})}));
end DryAir;
