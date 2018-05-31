within BuildingMpc.Examples.DesignOpt.DecisionVariablesRecords.Templates;
record ContinuousDecisionVariable "Continuous decision variable"
  parameter Real y "Normalised design variable";
  parameter Real y_min=0 "Lower range of design variable";
  parameter Real y_max=1 "Upper range of design variable";
  final parameter Real val=y_min+(y_max-y_min)*y "Design variable";
  parameter Real investment "Associated variable annual investment cost difference";
  parameter Real maintenance "Associated variable annual maintenance cost difference";
  parameter Real lifetime "Associated component lifetime in years before replacement occurs";
  parameter Real replacement "Replacement cost after 'lifetime' has expired";
  parameter Real interest "Annual depreciation/interest rate";
  parameter Real depreciationYears "";
  parameter Real cost = costReplacement + costInvestmentMaintenance;
  parameter Real costReplacement=
    investment*replacement*sum({(if mod(integer(j),integer(lifetime))==0 then 1/(1+interest)^j else 0) for j in 1:depreciationYears});
  parameter Real costInvestmentMaintenance=
    investment*(1+maintenance*sum({1/(1+interest)^j for j in 0:depreciationYears-1})) "Maintenance and investment cost using NPV";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ContinuousDecisionVariable;
