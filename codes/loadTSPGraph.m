function [G, Gnodes]= loadTSPGraph(fileName)
    % read the TSP graph from xml file and convert it to matlab structure 
    tsp = xml2struct(fileName);
    % convert graph structure to graph matrix 
    nodeNo = size(tsp.graph.vertex, 2);
    graphMat = zeros(nodeNo, nodeNo);
    names = zeros(nodeNo,1);
    for v=1 : nodeNo
        names(v) = v;
        vertex = tsp.graph.vertex(v);
        diag = 0;
        for e=1 : nodeNo-1
            edge = vertex{1}.edge(e);
            cost = str2double(edge{1}.Attributes.cost);
            if(v==e)
                diag = 1;
            end
            graphMat(v, e+diag) = cost;
        end
    end

    % convert graph matrix to matlab graph object
    Gnodes = cellstr(string(names));
    G = graph(graphMat, Gnodes);
end