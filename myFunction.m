function out = myFunction(op, varargin)
    switch op
        case 'vw2wheels'
            out = vw2wheels(varargin{:});
        case 'wheels2vw'
            out = wheels2vw(varargin{:});
        case 'qdot'
            out = qdot(varargin{:});
        case 'qupdate'
            out = qupdate(varargin{:});
        case 'control'
            out = control(varargin{:});
    end
end