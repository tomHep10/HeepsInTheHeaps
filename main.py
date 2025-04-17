from pathGraph import pathGraph

def main():
    i = int(input("1 for a*, 2 for d*, 0 for exit: "))
    graph = pathGraph()

    if i == 0:
        exit()
    if i == 1:
        graph._createAStarGraph(20, 20)
        main()
    if i == 2:
        main()
    else:
        print("Invalid input, please try again.")
        main()

if __name__ == "__main__":
    main()
