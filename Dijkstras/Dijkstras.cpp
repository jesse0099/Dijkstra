// Dijkstras.cpp : Dijkstras con matriz adyacente estatica. Grafo pesado y no orientado 
//(Adaptable)

#include <iostream>
#include <vector>
#include <algorithm>  
#include <string>

using namespace std;

#pragma region AVS
/*
	nv = non valid
	uv = unvisited
	nd = non directed
	idx = index
	sd = shortest distance
*/
#pragma endregion


const string NV_INDEX = "No se han indicado vertices validos";
const unsigned short int INFINITE = 9999;
const short int NV_VERTEX = -1;

#pragma region Vertices

class Vertex {
private:
	unsigned short int index;
	vector<vector<int>> nd_shortest_paths;
	string tag;

public:
	//Read nd_shortest_paths 
	//Return a vector where first[0] space is the shortest distance
	//The remaining spaces represent the vertexes indexes to achieve the shortest distance
	//Vertexes indexes go from start to end
	vector<int> nd_shortest_path(int end_idx) {
		vector<int> path;
		int objective = end_idx, previous;
		previous = objective;
		while (true) {
			previous = nd_shortest_paths[objective][1];
			if (previous == NV_VERTEX)
				break;
			path.insert(path.begin(),previous);
			objective = previous;
		}
		path.insert(path.begin(), nd_shortest_paths[end_idx][0]);
		path.push_back(end_idx);
		return path;
	}


	Vertex() {}
	Vertex(string p_tag) {
		tag = p_tag;
	}

	string get_tag() {
		return tag;
	}
	vector<vector<int>> get_nd_shortest_paths() {
		return nd_shortest_paths;
	}
	int get_index(){
		return index;
	}
	void set_index(int n_index) {
		index = n_index;
	}
	void set_tag(string n_tag) {
		tag = n_tag;
	}
	void set_nd_shortest_path(vector<vector<int>> p_nd_shortest_path) {
		nd_shortest_paths = p_nd_shortest_path;
	}
};

#pragma endregion

class Graph {

private:
	size_t vertexes;
	vector<Vertex> vertexes_list;
	vector<vector<int>>  graph; 
	void init_graph() {
		graph.resize(vertexes);/*Rows - Origin*/
		for (size_t i = 0; i < vertexes; i++) {
			vertexes_list[i].set_index(i); /*Assigning the indices of the vertices*/
			graph[i].resize(vertexes);/*Colums - End*/
			for (size_t j = 0; j < vertexes; j++) {
				graph[i][j] = 0;
			}
		}

	}
	vector<vector<int>> init_nd_sd_matrix(int vertex_idx) {
		int org_idx = vertex_idx;//Graph's Matrix row index

		//n(vertexes) * m(info cols (2))
		vector<vector<int>> info_table;
		info_table.resize(vertexes); //Vertexes rows

		for (size_t idx = 0; idx < vertexes; idx++) {
			idx == org_idx ? info_table[idx].push_back(0) : info_table[idx].push_back(INFINITE); //Shortest distance from start
			info_table[idx].push_back(NV_VERTEX); //Previous vertex
		}

		return info_table;
	}

public:

	void print_graph() {
		cout << "  ";
		for (auto x : vertexes_list)
			cout << x.get_tag() << " ";
		cout << endl;
		for (size_t i = 0; i < vertexes; i++)
		{
			for (size_t j = 0; j < vertexes; j++)
			{
				j == 0 ? cout << vertexes_list[i].get_tag() << " " : cout;
				cout << graph[i][j] << " ";
			}
			cout << endl;
		}
	}

	// :?) Perdon
	void foo() {}

	//If bidir is true: Non Directed graph (weight A->B [3] A<-B [3]) 
	//Directed graph  (weight A->B [3] A<-B [?])) 
	void add_edge(string vertex_a, string vertex_b, int weight, bool bidir = true) {
		int v_a = vertex_index(vertex_a), v_b = vertex_index(vertex_b);
		if (v_a > -1 && v_b > -1) {
			bidir ? (graph[v_a][v_b] = weight, graph[v_b][v_a] = weight):  graph[v_a][v_b] = weight;
			return;
		}
		cout << NV_INDEX << endl;
	}

	//Non Directed graph - Dijkstra shortest path. From vertex org to every other one (Vertex Tag)
	void nd_shortest_paths(string vertex, int vertex_idx = -1) {
		//Check if vertex_index returned something != -1
		int start_vertex = (vertex_idx != -1) ? vertex_idx : vertex_index(vertex);
		if (start_vertex != -1) {
			vector<vector<int>> info_table = init_nd_sd_matrix(start_vertex);
			
			int current_vertex;
			vector<int> visited_vertexes;
			vector<int> unvisited_vertexes;
			vector<int> current_neighbors;
			vector<int> current_neighbors_distances;
			
			//Loading all vertexes to unvisited list
			for (auto x : vertexes_list)
					unvisited_vertexes.push_back(x.get_index());

			//Visiting all vertexes
			while (unvisited_vertexes.size() != 0) {
				unvisited_vertexes.size() == vertexes ?  current_vertex = start_vertex : current_vertex = get_uv_known_sd_vertex(info_table,visited_vertexes);
				
				//No more reachable unvisited vertexes 
				if (current_vertex == NV_VERTEX)
					break;

				//Current vertex unvisited neighbors 
				current_neighbors = get_current_uv_neighbors(current_vertex, visited_vertexes);

				//Distances from start vertex to neighbors 
				current_neighbors_distances = get_current_uv_neighbors_sd(current_vertex, current_neighbors, info_table);

				update_sd(current_vertex,current_neighbors, current_neighbors_distances, info_table);

				visited_vertexes.push_back(current_vertex);
				unvisited_vertexes.erase(find(unvisited_vertexes.begin(), unvisited_vertexes.end(),current_vertex));
			}
			//Update vertex nd_shortest_paths
			vertexes_list[start_vertex].set_nd_shortest_path(info_table);
			return;
		}
		cout << NV_INDEX << endl;
	}
	
	//The shortest path from the start vertex to end vertex
	vector<int> nd_shortest_path(string start_vertex, string end_vertex) {
		int start_vertex_idx = vertex_index(start_vertex), end_vertex_idx;
		vector<int> error_return;
		if (start_vertex_idx != -1) {
			end_vertex_idx = vertex_index(end_vertex);
			if (end_vertex_idx != -1) {
				if (vertexes_list[start_vertex_idx].get_nd_shortest_paths().size() > 0) {
					//Read Table
					return vertexes_list[start_vertex_idx].nd_shortest_path(end_vertex_idx);
				}
				else {
					//Load Table
					nd_shortest_paths("NAN", start_vertex_idx);

					//Read Table
					return vertexes_list[start_vertex_idx].nd_shortest_path(end_vertex_idx);
				}
			}
			else {
				return error_return;
			}
		}
		else {
			return error_return;
		}
	}

	//Update non directed graph infotable 
	void update_sd(int pre_vertex,vector<int> neighbors, vector<int> neighbors_distances, vector<vector<int>> &info_table) {
		int idx = 0;
		for (auto x : neighbors) {
			if (neighbors_distances[idx] < info_table[x][0]) {
				info_table[x][0] = neighbors_distances[idx];
				info_table[x][1] = pre_vertex;
			}
			idx++;
		}
	}

	//Vertex row index given a  tag
	int vertex_index(string tag) {
		auto it = find_if(vertexes_list.begin(), vertexes_list.end(), [tag](Vertex obj) {return obj.get_tag() == tag; });
		if (it != vertexes_list.end())
			return (*it).get_index();
		return -1;
	}

	//Row index of the matrix's unvisited shortest registered distance (Dijkstra - Non Directed)

	int get_uv_known_sd_vertex(vector<vector<int>> info_table, vector<int> visited) {
		int minimum = INFINITE, min_index = NV_VERTEX, idx = 0;
		for (auto x : info_table) {
			minimum = (x[0] < minimum && find(visited.begin(), visited.end(), idx) == visited.end()) ? (min_index = idx, x[0])  : minimum;
			idx++;
		}
		return min_index;
	}

	//Current-vertex's neighbors
	vector<int> get_current_uv_neighbors(int current_idx, vector<int> visited) {
		vector<int> neighbours;
		for (size_t i = 0; i < vertexes; i++)
			graph[current_idx][i] > 0 && find(visited.begin(),visited.end(), i) == visited.end() ? neighbours.push_back(i) : foo();
		return neighbours;
	}

	//Shortest distances from start to neighbors  of the current vertex 
	vector<int> get_current_uv_neighbors_sd(int current_idx,vector<int> neighbors, vector<vector<int>> infotable) {
		//Start to current distance
		int start_to_current_sd = infotable[current_idx][0];
		vector<int> distances;
		//Start to current  + current to neighbor 
		for (unsigned int i = 0; i < neighbors.size(); i++)
			distances.push_back(graph[current_idx][neighbors[i]] + start_to_current_sd);
		return distances;
	}

	Graph(vector<Vertex> p_vertexes ) {
		vertexes_list = p_vertexes;
		vertexes = vertexes_list.size();
		init_graph();
	}
	Graph() {}


	vector<vector<int>> get_graph() {
		return graph;
	}
	vector<Vertex> get_vertexes_list() {
		return vertexes_list;
	}
	size_t get_vertexes() {
		return vertexes;
	}
	void set_graph(vector<vector<int>> p_graph) {
		graph = p_graph;
	}
	void set_vertexes_list(vector<Vertex> p_vertexes_list) {
		vertexes_list = p_vertexes_list;
	}
	void set_vertexes(int p_vertexes) {
		vertexes = p_vertexes;
	}

};




int main()
{
	Graph graph;
	vector<Vertex> vertexes;
	vertexes.push_back(Vertex("A"));
	vertexes.push_back(Vertex("B"));
	vertexes.push_back(Vertex("C"));
	vertexes.push_back(Vertex("D"));
	vertexes.push_back(Vertex("E"));
	vertexes.push_back(Vertex("F"));

	
	graph = Graph(vertexes);

	graph.add_edge("A", "B", 6, false);
	graph.add_edge("A","D",1);
	//graph.add_edge("B","D",2);
	graph.add_edge("B","E",2);
	graph.add_edge("B","C",5);
	//graph.add_nd_edge("B","F",5);
	graph.add_edge("C","E",5);
	graph.add_edge("E","D",1);

	//graph.nd_shortest_paths("A");

	//graph.get_vertexes_list()[graph.vertex_index("A")].get_nd_shortest_paths().size() > 0 ? cout << "Tabla Creada" << endl : cout << "Tabla No Creada" << endl;
	
	vector<int> path = graph.nd_shortest_path("B","A");
	int idx = 0;
	for (auto x : path) {
		idx == 0 ? cout << "Shortest Distance " << x << endl : (idx == path.size()-1) ? cout << graph.get_vertexes_list()[x].get_tag() << endl 
			: cout << graph.get_vertexes_list()[x].get_tag() <<"->";
		idx++;
	}
	cout << endl;

	graph.print_graph();

}

