
/*
 * This project was developed for the Introduction to Artificial Intelligence/Intelligent Systems
 * module COMP5280/8250 at University of Kent.
 *
 * The java code was created by Elena Botoeva (e.botoeva@kent.ac.uk) and
 * follows the structure and the design of the Pacman AI projects
 * (the core part of the project on search)
 * developed at UC Berkeley http://ai.berkeley.edu.
 */

/**
 * This file contains a skeleton of the search procedure. You need to implement
 * the concrete search functions.
 *
 * The printing is for outputting statistics about search, it is there to help you
 * see what you are running and how good your implementation is.
 *
 * Please only change the parts of the file you are asked to.  Look for the lines
 * that say
 *
 * //TODO: YOUR CODE HERE
 */

import java.util.*;

import static java.lang.System.out;

public class GraphSearch<S,A> {
    /**
     *
     * @param function string describing search strategy: dfs, bfs, greedy and astar
     * @param problem search problem instance, for instance, PacmanPositionSearchProblem, PacmanCornersProblem etc
     * @param heuristic can be null, not used for dfs and bfs
     * @param <S> states
     * @param <A> actions
     * @return
     */
    public static <S, A> Solution<S, A> search(String function, SearchProblem<S, A> problem, SearchHeuristic<S, A> heuristic) {
        Solution<S, A> solution;
        long start = System.currentTimeMillis();
        switch (function) {
            case "dfs":
                solution = depthFirstSearch(problem);
                break;
            case "bfs":
                solution = breadthFirstSearch(problem);
                break;
            case "greedy":
                solution = greedySearch(problem, heuristic);
                break;
            case "astar":
                solution = aStarSearch(problem, heuristic);
                break;
            case "gowest":
                if (problem instanceof PacmanPositionSearchProblem) {
                    solution = (Solution<S, A>) goWestSearch((PacmanPositionSearchProblem) problem);
                    break;
                }
            default:
                throw new RuntimeException("Unknown search strategy: " + function);
        }

        long end = System.currentTimeMillis();
        printSearchInfo(problem, solution, end - start);
        return solution;
    }

    private static Solution<PacmanPositionSearchState, PacmanAction> goWestSearch(PacmanPositionSearchProblem problem) {
        out.println("Hardcoded Go West strategy that works for testMaze");

        List<PacmanAction> actions = new ArrayList<>();
        PacmanPositionSearchState state = problem.getStartState();
        double cost = 0;

        while (!problem.isGoalState(state)) {
            problem.expand(state);
            cost += problem.getCost(state, PacmanAction.WEST);
            state = problem.getSuccessor(state, PacmanAction.WEST);
            actions.add(PacmanAction.WEST);
        }
        return new Solution<>(state, actions, cost);
    }

    private static <S, A> Solution<S, A> depthFirstSearch(SearchProblem<S, A> problem) {
        return graphSearch(problem, new Util.Stack<Node<S, A>>());
    }

    private static <S, A> Solution<S, A> breadthFirstSearch(SearchProblem<S, A> problem) {
        return graphSearch(problem, new Util.Queue<Node<S, A>>());
    }

    private static <S, A> Solution<S, A> greedySearch(SearchProblem<S, A> problem, SearchHeuristic<S, A> heuristic) {
        out.println("Search using Greedy algorithm and heuristic " + heuristic);

        //TODO: YOUR CODE HERE
        throw new RuntimeException("Not Implemented");
    }

    private static <S, A> Solution<S, A> aStarSearch(SearchProblem<S, A> problem, SearchHeuristic<S, A> heuristic) {
        Util.PriorityQueue<Node<S, A>> frontier = new Util.PriorityQueue<>(
                Comparator.comparingDouble(node -> node.fCost)
        );
        ArrayList<Node<S, A>> expanded = new ArrayList<>();

        // Add start state as new Node
        frontier.push(
                new Node(problem.getStartState(), new ArrayList(), 0.0, heuristic.value(problem.getStartState(), problem))
        );

        Node<S, A> currentNode, successorNode;
        List<A> actionsToCurrentNode;
        double newFCost;

        while (!frontier.isEmpty()) {
            currentNode = frontier.pop();
            if (problem.isGoalState(currentNode.getState())) {
                return new Solution<>(
                        currentNode.getState(),
                        currentNode.getActions(),
                        currentNode.getCost()
                );
            }

            if (!expanded.contains(currentNode)) {
                expanded.add(currentNode);
                problem.expand(currentNode.getState());

                for (A action: problem.getActions(currentNode.getState())) {
                    actionsToCurrentNode = new ArrayList<>(currentNode.getActions());
                    actionsToCurrentNode.add(action);

                    successorNode = new Node<>(
                            problem.getSuccessor(currentNode.getState(), action),
                            actionsToCurrentNode,
                            currentNode.getCost() + problem.getCost(currentNode.getState(), action),
                            heuristic.value(
                                    problem.getSuccessor(currentNode.getState(), action), problem
                            )
                    );

                    boolean alreadyExpanded = false;
                    for (Node<S, A> expNode : expanded) {
                        if (expNode.getState().equals(successorNode.getState())) {
                            alreadyExpanded = true;
                        }
                    }

                    if (!alreadyExpanded) {
                        frontier.push(successorNode);
                    }
                }

            }
        }

        return null;
    }

    private static <S, A> Solution<S, A> graphSearch(SearchProblem<S, A> problem, Util.Frontier<Node<S,A>> frontier) {
        ArrayList<Node<S, A>> expanded = new ArrayList<>();

        // Add start state as new Node
        frontier.push(
                new Node(problem.getStartState(), new ArrayList(), 0.0)
        );

        Node<S, A> currentNode, successorNode;
        List<A> actionsToCurrentNode;

        while (!frontier.isEmpty()) {
            currentNode = frontier.pop();
            if (problem.isGoalState(currentNode.getState())) {
                return new Solution<>(
                        currentNode.getState(),
                        currentNode.getActions(),
                        currentNode.getCost()
                );
            }

            if (!expanded.contains(currentNode)) {
                expanded.add(currentNode);
                problem.expand(currentNode.getState());

                for (A action: problem.getActions(currentNode.getState())) {
                    actionsToCurrentNode = new ArrayList<>(currentNode.getActions());
                    actionsToCurrentNode.add(action);

                    successorNode = new Node<>(
                            problem.getSuccessor(currentNode.getState(), action),
                            actionsToCurrentNode,
                            currentNode.getCost() + problem.getCost(currentNode.getState(), action)
                    );

                    // I might rework this later (using equals or compare to), but currently
                    // contains() will not work because a node with West->East is counted as
                    // the same as the start node even thought both of their states are (5,5),
                    // this works around that to make sure the same state space doesn't get
                    // expanded twice
                    boolean alreadyExpanded = false;
                    for (Node<S, A> expNode : expanded) {
                        if (expNode.getState().equals(successorNode.getState())) {
                            alreadyExpanded = true;
                        }
                    }

                    if (!alreadyExpanded) {
                        frontier.push(successorNode);
                    }
                }

            }
        }

        return null;
    }

    public static <S, A> void printSearchInfo(SearchProblem<S, A> searchProblem,
                                Solution<S, A> solution,
                                long runtime) {
        out.println("Search Problem: " + searchProblem.getClass().getName());
        if (solution != null) {
            out.println("Solution found in " + (double) runtime / 1000 + "s");
            out.println("Solution cost " + solution.pathCost);
        } else {
            out.println("No solution found in " + (double) runtime / 1000 + "s");
        }
        out.println("Expanded count " + searchProblem.getExpandedCount());
        out.println("");
    }
}

/**
 * You might want to implement this data structure to represent nodes of a search tree.
 * You could store objects of this class in the frontier.
 */
class Node<S, A> implements Comparable<Node<S, A>> {

    double cost;
    private S state;
    private List<A> actions;

    // for A*
    double fCost;

    public Node(S state, List<A> actions, double cost) {
        this.state = state;
        this.actions = actions;
        this.cost = cost;
    }

    // for A*
    public Node(S state, List<A> actions, double cost, double fCost) {
        this.state = state;
        this.actions = actions;
        this.cost = cost;
        this.fCost = fCost;
    }

    public double getCost() {
        return cost;
    }

    public S getState() {
        return state;
    }

    public List<A> getActions() {
        return actions;
    }

    @Override
    public int compareTo(Node<S, A> otherNode) {
        //TODO: YOUR CODE HERE
        throw new RuntimeException("Not Implemented");
    }

    /**
     * Useful for debugging
     */
    @Override
    public String toString() {
        //TODO: YOUR CODE HERE
        throw new RuntimeException("Not Implemented");
    }
}


