within BuildingMpc.Examples.DesignOpt.DecisionVariablesRecords.Templates;
partial record DiscreteDecisionVariable "Decision variable"
  parameter Real[numOpts] investment "Associated variable initial investment cost";
  parameter Real[numOpts] maintenance "Associated variable annual maintenance cost as fraction of investment cost";
  parameter Integer[numOpts] lifetime "Associated variable lifetime in years before replacement occurs";
  parameter Real[numOpts] replacement "Replacement cost after 'lifetime' has expired as fraction of investment cost";
  parameter Real[numOpts] interest "Annual depreciation/interest rate";
  parameter Integer depreciationYears = 25 "Duration for NPV calculations";
  parameter Integer numOpts(start=1,min=1) "Number of optimisation variables";
  parameter Integer i(start=1, min=1, max=numOpts) "Decision variable index";
  parameter Real cost = costReplacement + costInvestmentMaintenance;
  parameter Real depreciation = sum({1/(1+interest[i])^j for j in 0:depreciationYears-1});
  parameter Real costReplacement=investment[i]*replacement[i]*floor((depreciationYears-0.001)/lifetime[i]);

  parameter Real costInvestmentMaintenance=investment[i]*(1 + maintenance[i]*depreciation) "Maintenance and investment cost using NPV";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end DiscreteDecisionVariable;
