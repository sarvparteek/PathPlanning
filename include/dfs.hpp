/*!
 * @brief: Implements depth first search
 * @author: Sarv Parteek Singh
 * @date: June-29-2020
 * @details: Implementation inspired by https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
 */

#ifndef DFS_HPP
#define DFS_HPP

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


private:
    /*!
     * @brief Utility called recursively to assist Depth First Search
     */
    void DfsUtility(T const &vertex, std::vector<bool> &visited);

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
void Graph<T>::DfsUtility(T const &vertex, std::vector<bool> &visited )
{
    visited[getMapIndex(m_adj_list_map, vertex)] = true;
    std::cout << "Visited vertex " << vertex << std::endl;

    auto &m_adj_list_at_vertex = m_adj_list_map[vertex];
    for (auto it = m_adj_list_at_vertex.begin(); it != m_adj_list_at_vertex.end(); ++it)
    {
        if (!visited.at(getMapIndex(m_adj_list_map, *it))) // it: iterator to a vertex in the adjacency list
        {
            DfsUtility(*it, visited);
        }
    }
}

template<typename T>
void Graph<T>::executeDfs(const T &vertex)
{
    std::vector<bool> visited; // flag to indicate visited status for each vertex. Index here is the same as the index
                               // in the map.
    visited.resize(m_adj_list_map.size());
    std::fill(visited.begin(), visited.end(), false); // no vertex has been visited to begin with
    DfsUtility(vertex, visited);
}

template <typename T>
void Graph<T>::executeDfs()
{
    std::vector<bool> visited; // flag to indicate visited status for each vertex. Index here is the same as the index
    // in the map.
    visited.resize(m_adj_list_map.size());
    std::fill(visited.begin(), visited.end(), false); // no vertex has been visited to begin with

    for (auto it = m_adj_list_map.cbegin(); it != m_adj_list_map.cend(); ++it)
    {
        if (!visited.at(getMapIndex(m_adj_list_map, it->first))) // it: iterator to an std::pair<T, std::list>
        {
            DfsUtility(it->first, visited);
        }
    }
}

template<typename T>
std::size_t Graph<T>::getMapIndex(const std::map<T, std::list<T> > &m, const T &element)
{
    std::size_t idx = std::distance(m.begin(), m.find(element));
    return idx;
}

// TODO - June-30-2020: Move the following to a GTEST Framework under a new (test/) folder.
void testDFS()
{
    Graph<int> g_int;
    g_int.addEdge(0, 1);
    g_int.addEdge(0, 2);
    g_int.addEdge(1, 2);
    g_int.addEdge(2, 0);
    g_int.addEdge(2, 3);
    g_int.addEdge(3, 3);
    std::cout << "Depth First Search traversal starting for int-based graph, from vertex 2 : " << std::endl;
    g_int.executeDfs(2);

    Graph<char> g_char;
    g_char.addEdge('0', '1');
    g_char.addEdge('0', '2');
    g_char.addEdge('1', '2');
    g_char.addEdge('2', '0');
    g_char.addEdge('2', '3');
    g_char.addEdge('3', '3');
    std::cout << "Depth First Search traversal starting for char-based graph, from vertex '2' : " << std::endl;
    g_char.executeDfs('2');

    Graph<float> g_float;
    g_float.addEdge(0.1, 1.1);
    g_float.addEdge(0.1, 2.1);
    g_float.addEdge(1.1, 2.1);
    g_float.addEdge(2.1, 0.1);
    g_float.addEdge(2.1, 3.1);
    g_float.addEdge(3.1, 3.1);
    std::cout << "Depth First Search traversal starting for float-based graph, from vertex 2.1 : " << std::endl;
    g_float.executeDfs(2.1);

    Graph<int> g_int_disc;
    g_int_disc.addEdge(0, 1);
    g_int_disc.addEdge(0, 2);
    g_int_disc.addEdge(1, 2);
    g_int_disc.addEdge(2, 0);
    g_int_disc.addEdge(2, 3);
    g_int_disc.addEdge(3, 3);

    std::cout << "Depth First Search traversal in an int-based disconnected graph" << std::endl;
    g_int_disc.executeDfs();

}

#endif

