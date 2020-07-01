/*!
 * @brief: Implements depth first search
 * @author: Sarv Parteek Singh
 * @date: June-29-2020
 * @details: Implementation inspired by
 *           https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
 *           https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/
 */

#ifndef BREADTH_DEPTH_FIRST_SEARCH_HPP
#define BREADTH_DEPTH_FIRST_SEARCH_HPP

#include <list>      // for list
#include <map>       // for map
#include <vector>    // for vector
#include <iostream>  // for cout, endl
#include <algorithm> // for fill, distance

template <typename T>
class Graph
{
public:
    Graph()
    {}

    /*!
     * @brief Add an edge to the graph
     * @param[in] vertex : Starting vertex
     * @param[in] adj_vertex: Vertex adjoining the starting vertex
     */
    void addEdge(T const& vertex, T const& adj_vertex);

    /*!
     * @brief Explores vertices using Depth First Search starting from the input vertex, when the graph is connected.
     * @param vertex
     */
    void executeDfs(T const &vertex);

    /*!
     * @brief Explores vertices using Depth First Search when the graph is disconnected
     */
    void executeDfs();

    /*!
     * @brief Explores vertices using Breadth First Search starting from the input vertex
     */
    void executeBfs(T const &vertex);


private:
    /*!
     * @brief Implements crux of Depth First Search, via recursive calls
     */
    void recurDfs(T const &vertex, std::vector<bool> visited);

    /*!
     * @brief Creates a vector to keep track of visited vertices
     * @return Vector of size equal to number of vertices with all entries set to false
     */
    std::vector<bool> createVisitedTracker();

    /*!
     * @brief Gives the index of the map where the input element is located (assuming unique keys in the map)
     * @param[in] m Map to be searched
     * @param[in] element Key in the map whose index is required
     * @return Index value
     */
    std::size_t getMapIndex(std::map<T, std::list<T>> const &m, T const &element);

    std::map<T, std::list<T>> m_adj_list_map; /*! Adjacency list: this is a map between a vertex and the corresponding
                                               *  adjacency list i.e. list of all elements adjacent to the vertex.
                                               *  Ordered map (instead of unordered map) helps with indexing.
                                               * */
};

// TODO - 2020-06-30: Move the following to a source file under src (remember that explicit template instantiation
// would be required).
// Header:
// extern template class Graph<char>
// extern template class Graph<int>, ...and the like
// Source:
// template class Graph<char>
// template class Graph<int> ... and the like

template<typename T>
void Graph<T>::addEdge(T const& vertex, T const& adj_vertex)
{
    m_adj_list_map[vertex].push_back(adj_vertex);
}

template<typename T>
void Graph<T>::executeDfs(T const &vertex)
{
    recurDfs(vertex, createVisitedTracker());
}

template <typename T>
void Graph<T>::executeDfs()
{
    auto visited = createVisitedTracker();
    for (auto it = m_adj_list_map.cbegin(); it != m_adj_list_map.cend(); ++it)
    {
        if (!visited.at(getMapIndex(m_adj_list_map, it->first))) // it: iterator to an std::pair<T, std::list>
        {
            recurDfs(it->first, visited);
        }
    }
}

template <typename T>
void Graph<T>::executeBfs(T const &vertex)
{
    auto visited = createVisitedTracker();
    visited.at(getMapIndex(m_adj_list_map, vertex)) = true;

    std::list<T> queue{vertex}; // Create a queue to keep track of vertices explored and initialize it with the input
                                // vertex

     while (!queue.empty())
     {
         auto l_vertex = queue.front(); // local vertex (potentially different from input argument vertex)
         queue.pop_front();
         std::cout << "Visited vertex " << l_vertex << std::endl;
         auto &l_adj_list_at_l_vertex = m_adj_list_map[l_vertex];
         for (auto it = l_adj_list_at_l_vertex.cbegin(); it != l_adj_list_at_l_vertex.cend(); ++it)
         {
             auto idx_for_l_vertex = getMapIndex(m_adj_list_map, *it); // it: iterator to a vertex in the adjacency list
             if (!visited.at(idx_for_l_vertex))
             {
                 visited.at(idx_for_l_vertex) = true;
                 queue.push_back(*it);
             }
         }
     }
}

template<typename T>
void Graph<T>::recurDfs(T const &vertex, std::vector<bool> visited )
{
    visited.at(getMapIndex(m_adj_list_map, vertex)) = true;
    std::cout << "Visited vertex " << vertex << std::endl;

    auto &l_adj_list_at_vertex = m_adj_list_map[vertex];
    for (auto it = l_adj_list_at_vertex.begin(); it != l_adj_list_at_vertex.end(); ++it)
    {
        if (!visited.at(getMapIndex(m_adj_list_map, *it))) // it: iterator to a vertex in the adjacency list
        {
            recurDfs(*it, visited);
        }
    }
}

template<typename T>
std::vector<bool> Graph<T>::createVisitedTracker()
{
    std::vector<bool> visited; // flag to indicate visited status for each vertex. Index here is the same as the index
                               // in the map.
    visited.resize(m_adj_list_map.size());
    std::fill(visited.begin(), visited.end(), false); // no vertex has been visited to begin with
    return visited; // Return value optimization will ensure that std::move is used
}

template<typename T>
std::size_t Graph<T>::getMapIndex(const std::map<T, std::list<T> > &m, const T &element)
{
    std::size_t idx = std::distance(m.begin(), m.find(element));
    return idx;
}

// TODO - June-30-2020: Move the following to a GTEST Framework under a new (test/) folder.
void testBdfs()
{
    Graph<int> g_int;
    g_int.addEdge(0, 1);
    g_int.addEdge(0, 2);
    g_int.addEdge(1, 2);
    g_int.addEdge(2, 0);
    g_int.addEdge(2, 3);
    g_int.addEdge(3, 3);
    std::cout << "\nDepth First Search traversal starting for int-based graph, from vertex 2 : " << std::endl;
    g_int.executeDfs(2);
    std::cout << "Breadth First Search traversal starting for int-based graph, from vertex 2 : " << std::endl;
    g_int.executeBfs(2);

    Graph<char> g_char;
    g_char.addEdge('0', '1');
    g_char.addEdge('0', '2');
    g_char.addEdge('1', '2');
    g_char.addEdge('2', '0');
    g_char.addEdge('2', '3');
    g_char.addEdge('3', '3');
    std::cout << "\nDepth First Search traversal starting for char-based graph, from vertex '2' : " << std::endl;
    g_char.executeDfs('2');
    std::cout << "Breadth First Search traversal starting for char-based graph, from vertex '2' : " << std::endl;
    g_char.executeBfs('2');

    Graph<float> g_float;
    g_float.addEdge(0.1, 1.1);
    g_float.addEdge(0.1, 2.1);
    g_float.addEdge(1.1, 2.1);
    g_float.addEdge(2.1, 0.1);
    g_float.addEdge(2.1, 3.1);
    g_float.addEdge(3.1, 3.1);
    std::cout << "\nDepth First Search traversal starting for float-based graph, from vertex 2.1 : " << std::endl;
    g_float.executeDfs(2.1);
    std::cout << "Breadth First Search traversal starting for float-based graph, from vertex 2.1 : " << std::endl;
    g_float.executeBfs(2.1);

    Graph<int> g_int_disc;
    g_int_disc.addEdge(0, 1);
    g_int_disc.addEdge(0, 2);
    g_int_disc.addEdge(1, 2);
    g_int_disc.addEdge(2, 0);
    g_int_disc.addEdge(2, 3);
    g_int_disc.addEdge(3, 3);

    std::cout << "\nDepth First Search traversal in an int-based disconnected graph" << std::endl;
    g_int_disc.executeDfs();

}

#endif // BREADTH_DEPTH_FIRST_SEARCH_HPP

