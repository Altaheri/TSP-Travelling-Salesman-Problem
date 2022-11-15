
% read the TSP graph from xml file and convert it to matlab graph object
[G, Gnodes] = loadTSPGraph("att48_.xml");
%[G, Gnodes] = loadTSPGraph("a280_.xml");
%[G, Gnodes] = loadTSPGraph("data2.xml");
tic
% create an initial state (initial tour)
curState = createInitialState(G);

% navigate through the permutations of the current state (current tour).
% one permutation is performed by changing the positions of two nodes
% in the current tour (or deleting two edges and adding two).
% The new and current tours are called 2-opt neighbours.
% For each new tour, we compare the cost of the current and new tours.
% If the new tour has lower cost,
% we change the current tour by the new tour, 
% If the current tour does not change for many itreations (e.g. 500
% itreation), we my terminate the local seach and return the current tour
% as a potential maxima (minimum cost)

% disp( [curState.node , sum(curState.distance)]);
iterations = 0;
while(true)
    disp( [num2str(iterations) , '   ', num2str(sum(curState.distance))]);
    
    newState = getNextSolution(G, curState);
    if(newState.distance == curState.distance)
        break;
    end
    curState = newState;
    iterations = iterations + 1;
end
    
toc

function curState = getNextSolution(G, curState)
    iterations = 1;
    for p1 = 1 : size(curState.node,2)-1
        for p2 = p1+1 : size(curState.node,2)
            newTour = curState.node;
            tempNode = newTour(p1);
            newTour(p1)= curState.node(p2);
            newTour(p2)= tempNode;
            newState = getTourState(G, newTour');
            if( sum(newState.distance) < sum(curState.distance) )
                curState = newState;
            end

            iterations = iterations + 1;
        end
    end
end

function initState = createInitialState(G)
    tour = table2cell(G.Nodes);
    initState = getTourState(G, tour);
end

function state = getTourState(G, tour)
    for idx=1 : size(tour,1) 
        curNode = tour(idx);
        if(idx == size(tour,1))
            nextNode = cell2mat(tour(1));
        else
            nextNode = cell2mat(tour(idx+1));
        end
        state.node(idx) = curNode;
        state.distance(idx) = G.Edges.Weight(findedge(G, curNode ,nextNode));
    end
end
