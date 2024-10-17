classdef subClass < superClass
    properties
        prop2
    end
    
    methods
        function obj = subClass(val1, val2)
            % Call the superclass constructor to initialize prop1
            obj = obj@superClass(val1);
            % Initialize prop2 in the subclass
            obj.prop2 = val2;
        end
    end
end