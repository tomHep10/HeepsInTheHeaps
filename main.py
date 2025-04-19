from pathGraph import pathGraph

def main():
    i = int(input("1 for a*, 2 for d*, 0 for exit: "))
    graph = pathGraph()

    if i == 0:
        exit()
    if i == 1:
        graph._createAStarGraph(5, 5)
        #keeping start at (0,0) and end at (4,4) for now
        path = graph.aStarShortestPath((0,0), (4,4), "Manhattan")
        if path is not None:
            graph.aStarPath = path
            for x,y  in path:
                print((x,y), end = " ")
        main()
    if i == 2:
        main()
    else:
        print("Invalid input, please try again.")
        main()

if __name__ == "__main__":
    main()
