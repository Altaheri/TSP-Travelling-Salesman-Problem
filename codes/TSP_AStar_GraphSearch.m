
% read the TSP graph from xml file and convert it to matlab graph object 
[G, Gnodes] = loadTSPGraph("att48_.xml");
%[G, Gnodes] = loadTSPGraph("a280_.xml");
%[G, Gnodes] = loadTSPGraph("data2.xml");
%[G, Gnodes] = loadTSPGraph("burma14_.xml");

solutions =[];
time = [];
tic


for idx=2 : size(Gnodes,1)  % select a node to be as start node
    tic
    open=[];
    closed=[];
    % initialize the startt node 
    strnode = initializeStartNode(G, Gnodes{idx});
    % add the start node to the open list ( finge list)
    open = [open; strnode];
    solution = [];  % the goal strarting from node i
    Max_depth=0;
    no_expanded_nodes = 0;    
    
    while(~isempty(open))
        no_expanded_nodes=no_expanded_nodes+1;
        % get the minimum cost in the fringe
        costs = cell2mat({open.cost});
        depths = cell2mat({open.depth});
        % get the nodes that have the minimum cost 
        minCNodes = find(costs==min(costs));
        % if there are more than one node with the same cost, get deepest
        % one
        [val, index] = max(depths(minCNodes));
        minidx = minCNodes(index);
        currentNode = open(minidx);

        % display the state of searching 
        if( Max_depth < max(cell2mat({open.depth})) )
            Max_depth = max(cell2mat({open.depth}));
            disp( ['Node: ', currentNode.name, ...
                ',  Depth: ', num2str(currentNode.depth), ... 
                ',  Max-depth: ', num2str(max(cell2mat({open.depth}))), ...
                ',  no_expanded_nodes: ', num2str(no_expanded_nodes), ...
                ',  Fringe-size: ', num2str(size(open,1)), ...
                ' ,  Elapsed time is ', num2str(toc) ,' seconds.'] );
        end
        
        open(minidx)=[];
        if(~ismember(currentNode.name, {open.name}))
            closed = [closed; currentNode];
        end


        % check if the node is the goal.
        solution = isGoal(currentNode, G);
        if (~isempty(solution))
            break;
        end
        
        % expand the node
        nodelist = expandNode(G, strnode.name, currentNode);

        % For graph search, we expand the node only if it was not expanded before 
        for i=1:size(nodelist, 1)
            if (isempty(closed))
                val2 = 0;
            else
                [val2, id2] = ismember(nodelist(i).name,{closed.name});
            end

            if(val2 && (nodelist(i).cost<=closed(id2).cost))
                closed(id2)=[];
                open = [open; nodelist(i)];
            elseif(val2 && (nodelist(i).cost>closed(id2).cost))
                if((nodelist(i).depth > closed(id2).depth))
                    closed(id2)=[];
                    open = [open; nodelist(i)];
                end                    
            else
                id1 = find(ismember({open.name}, nodelist(i).name));
                if (isempty(id1))
                    open = [open; nodelist(i)];
                else
                    addnode = false;
                    removeidxs = [];
                    for s=1:size(id1,2)
                        if((nodelist(i).cost <= open(id1(s)).cost))
                            addnode = true;
                            if( (nodelist(i).cost < open(id1(s)).cost) && ...
                                (nodelist(i).depth > open(id1(s)).depth))
                                removeidxs = [removeidxs; id1(s)];
                            end
                        elseif( (nodelist(i).cost > open(id1(s)).cost) && ...
                                (nodelist(i).depth >= open(id1(s)).depth) )
                            addnode = true;
                         end
                    end
                    if(~isempty(removeidxs))
                        removeidxs = sort(removeidxs, 'descend');
                    end
                    for s=1:size(removeidxs,1)
                        open(removeidxs(s))=[];
                    end
                    if(addnode)
                        open = [open; nodelist(i)];
                    end
                end
            end
        end
    end
    if (~isempty(solution))
        solution.cost
        solution.path(2)
        solutions = [solutions; solution];
        time = [time; toc];
    end
end 
sum(cell2mat({solutions.cost}))/size(solutions,1)
toc
function node = initializeStartNode(G, startNode)
    % remove the first node from the the graph and add it to the open list
    % for the first node we have to 

    unvisitedGraph = rmnode(G, startNode);
    h_n = estimateDist(G, unvisitedGraph, startNode, startNode);
    node.name = startNode;
    node.h_n = h_n;
    node.g_n = 0;
    node.depth = 0;
    node.cost = node.h_n + node.g_n;
    node.parent = [];
end


% estimateDist function 
% estimate the distance from a current Node 'currentNode' to the end node 'startNode'
function cost = estimateDist(G, unvisitedGraph, startNode, currentNode)
    % caculate Minimum spanning tree using Prim痴 algorithm
    T = minspantree(unvisitedGraph); 
    % estimated the distance to visit all unvisited nodes using MST heuristic
    h_n_MST = sum(T.Edges.Weight);
    
    unvisitedNs = table2cell(unvisitedGraph.Nodes);
    % caculate the minimum distance from an unvisited node to the current node
    idx_to_curr_node = findedge(G, currentNode, unvisitedNs);
    mindist_to_curr_node = min(G.Edges.Weight(idx_to_curr_node));

    % caculate the minimum distance from an unvisited node to the start node
    idx_to_str_node = findedge(G, startNode, unvisitedNs);
    mindist_to_str_node = min(G.Edges.Weight(idx_to_str_node));
    
    cost = h_n_MST + mindist_to_str_node + mindist_to_curr_node;
end

% expandNode function 
% returns all the successors of a node 
function nodelist = expandNode(G, startNode, currentNode)
    % For TSP problem (each node is visited only once)
    % here we just get the nodes that are not already in the current node path
    unvisitedGraph = G;
    node1 = currentNode;
    while(~isempty(node1))
        unvisitedGraph = rmnode(unvisitedGraph, node1.name);
        node1 = node1.parent;
    end
    % the expanded nodes
    unvisitedNs = table2cell(unvisitedGraph.Nodes);
    % the expanded nodes as structure (to be returned)
    nodelist=[]; 
    
    % estimate cost for the expanded nodes
    for i=1:size(unvisitedNs,1)
        expandedNode = unvisitedNs{i}; % expanded node name as a string (char array)
        % if the node is leaf (the last node in the path), the heuristic is  
        % the cost of connectin this node to the start node
        if (size(unvisitedNs,1)==1)
            idx_to_str_node = findedge(G, startNode, expandedNode);
            h_n = G.Edges.Weight(idx_to_str_node);
        else
            unvistg = rmnode(unvisitedGraph, expandedNode);
            % h_n = cost of the MST of the subgraph of expandedNode +
            % minimum cost to connecte MST to expandedNode +
            % minimum cost to connecte MST to start node 
            h_n = estimateDist(G, unvistg, startNode, expandedNode);
        end
        
        % create the successor node as structure
        node.name = expandedNode; % the successor node
        node.h_n = h_n; % the heuristic of the successor node
        % the cost (edge value) of connecting the current node with its 
        % successor (the expanded node)
        g_n = G.Edges.Weight(findedge(G, node.name ,currentNode.name));
        % the cost of connecting the current node to the first node
        % (currentNode.g_n). node.g_n is the cost of the successor node to
        % the start node
        node.g_n = currentNode.g_n + g_n;
        % the estimated cost from the current from start node to the goal 
        % node throw the successor node
        node.depth = currentNode.depth+1;
        node.cost = node.h_n + node.g_n;          
        node.parent = currentNode;
        nodelist = [nodelist; node];
    end
end

% isGoal function 
% Test if the node is the goal node (if the goal is satisfied)
function solution = isGoal(node, G)
    path=[];
    snode=[];
    cost=node.cost;
    while(~isempty(node))
        path = [path; {node.name}];
        G = rmnode(G, node.name);
        snode = {node.name};
        node = node.parent;
    end
    if (isempty(G.Nodes))
        path = [snode; path];
        solution.path=path;
        solution.cost=cost;
    else
        solution = [];
    end
end