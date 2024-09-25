#include <iostream>
#include <limits.h>
#include <vector>
#include <algorithm>
#include <map>
#include <fstream>
//#include <chrono>
#include <bits/stdc++.h>

using namespace std;

struct info {
    long long int year;
    long long int hour;
    long long int cost;
};

void printSolution(vector<long long int>& peso) {
    
    for (long long int i = 0; i < peso.size() ; i++) {
        //cout << peso[i] << endl;
        printf("%lld\n", peso[i]);
    }
}

long long int minDistance(const vector<long long int>& peso, const vector<bool>& visited) {
    long long int min = LLONG_MAX, min_index;
    for (long long int v = 0; v < peso.size(); v++) {
        if (!visited[v] && peso[v] <= min) {
            min = peso[v];
            min_index = v;
        }
    }
    return min_index;
}

//auxiliar para o krustal
void makeSet(long long int n, vector<long long int>& parent, vector<long long int>& rank){
    for(long long int i=0;i<n;i++){
        parent[i] = i;
        rank[i] = 0;
    }
}
    
//auxiliar para o krustal
long long int findSet(vector<long long int>& parent, long long int v){
    if(v != parent[v]){
        parent[v] = findSet(parent, parent[v]);
    }
    return parent[v];
}
 
//auxiliar para o krustal       
void unionSets(vector<long long int>& parent, vector<long long int>& rank,long long int u, long long int v){
    long long int ind_u = findSet(parent, u);
    long long int ind_v = findSet(parent, v);

    if(ind_u != ind_v){
        if(rank[ind_u] > rank[ind_v]){
            parent[ind_v] = ind_u;
        }else if(rank[ind_u] < rank[ind_v]){
            parent[ind_u] = ind_v;
        }else{
            parent[ind_v] = ind_u;
            rank[ind_u]++;
        }
    }
}

//algoritmo de kruskal pra AGM. Usando map para evitar matriz de adjacencia declarada de forma estática.
//adaptada para encontrar o primeiro ano em que é possivel visitar qualquer cidade
void kruskalMST(long long int V, map<pair<long long int,long long int>,info>& graph) {

    vector<long long int> parent(V,0);
    vector<long long int> rank(V,0);

    makeSet(V, parent, rank);
    //vector<pair<long long int, pair<long long int, long long int>>> edges;
    vector<pair<pair<long long int,long long int>,info>> edges(V);
    //vetor de (edge(u,v), info)

    //for (long long int i = 0; i < V; i++) {
    //    for (long long int j = i + 1; j < V; j++) {
    //        if (graph[{i,j}].year > 0) {
    //            edges.push_back({ graph[{i,j}].cost, { i, j } });
    //        }
    //    }
    //}
    //melhoria
    for(auto& it : graph){
        edges.push_back(it);
    }
//ordeno as arestas
    sort(edges.begin(), edges.end(), [&](pair<pair<long long int,long long int>,info> &a , pair<pair<long long int,long long int>,info> &b){
        return a.second.year < b.second.year;
    });

    long long int min_cost = 0;
    long long int set_u, set_v, cost, u, v;

    
    //vetor de (edge(u,v), info)
    for (auto &edge : edges) {
        cost = edge.second.year;
        u = edge.first.second;
        v = edge.first.first;

        set_u = findSet(parent, u);
        set_v = findSet(parent, v);
        
        //se pertencerem a conjuntos diferentes, faça union
        if (set_u != set_v) {
            if(edge.second.year > min_cost)
                min_cost = edge.second.year;
            unionSets(parent, rank, set_u, set_v);
        }
    }
    //cout << min_cost << endl;
    printf("%lld\n", min_cost);
}

//calcula a distancia de um vertice raiz - palacio real) para todos os outros vertice s
void dijkstra(map<pair<long long int, long long int>, info>& graph, long long int V, long long int first_year, vector<vector<long long int>>& Adj) {
    long long int src = 0, year_max = 0;
    vector<long long int> peso(V, LLONG_MAX), year(V, LLONG_MAX);
    vector<bool> visited(V, false);

    peso[src] = 0;
    year[src] = 0;
    long long int u;
    //Dios mio
    priority_queue<pair<long long int, long long int>, vector<pair<long long int, long long int>>, greater<pair<long long int, long long int>>> pq;
    pq.push({0, src});

    while (!pq.empty()) {
        u = pq.top().second;
        pq.pop();

        if (visited[u])
            continue;

        visited[u] = true;

        for (long long int v = 0; v < Adj[u].size(); v++) {
            long long int aux = Adj[u][v];

            if (peso[u] + graph[{u, aux}].hour <= peso[aux]) {
                peso[aux] = peso[u] + graph[{u, aux}].hour;
                year[aux] = max(year[u], graph[{u, aux}].year);

                pq.push({peso[aux], aux});
            }
        }
    }

    for (long long int i = 0; i < year.size(); i++)
        if (year[i] > year_max)
            year_max = year[i];
            
    printSolution(peso);
    printf("%lld\n", year_max);
}

void kruskalMST_min_cost(long long int V, map<pair<long long int,long long int>,info>& graph) {

    vector<long long int> parent(V,0);
    vector<long long int> rank(V,0);

    makeSet(V, parent, rank);
    //vector<pair<long long int, pair<long long int, long long int>>> edges;
    vector<pair<pair<long long int,long long int>,info>> edges(V);
    //vetor de (edge(u,v), info)
    for(auto& it : graph){
        edges.push_back(it);
    }

    sort(edges.begin(), edges.end(), [&](pair<pair<long long int,long long int>,info> &a , pair<pair<long long int,long long int>,info> &b){
        return a.second.cost < b.second.cost;
    });

    long long int min_cost = 0;
    long long int set_u, set_v, cost, u, v;

    //vetor de (edge(u,v), info)
    for (auto &edge : edges) {
        cost = edge.second.cost;
        u = edge.first.second;
        v = edge.first.first;

        set_u = findSet(parent, u);
        set_v = findSet(parent, v);

        if (set_u != set_v) {
            min_cost += cost;
            unionSets(parent, rank, set_u, set_v);
        }
    }
    //cout << min_cost << endl;
    printf("%lld\n", min_cost);
}

int main() {

    /*
1. solução: Kruskal com o palacio real de raiz
    (a) solução: Ao passar por cada via, salvar a distancia(hour) sempre que encontrar uma maior que a atual
2. kruskal com peso = ano
3. kruskal com peso = custo
    */
    //FILE* inputFile = fopen("in.txt", "r");
    //if (inputFile == nullptr) {
    //    cerr << "erro" << endl;
    //    return 1;
    //}

    long long int vilas_count, conections_count;
    //cin  >> vilas_count >> conections_count;
    scanf("%lld %lld", &vilas_count, &conections_count);
    //vector<vector<long long int>> Adj(vilas_count);
    vector<vector<long long int>> ADJ_NEW(vilas_count);
    //vector<vector<info>> graph(vilas_count, vector<info>(10));
    //vector<vector<info>> graph(vilas_count, vector<info>(vilas_count, { 0, 0, 0 }));
    map<pair<long long int,long long int>,info> graph;
    //vector<vector<info>> graph;

    long long int vila1, vila2, year, hour, cost;
    long long int first_year = -1;
    struct info aux1, aux2;
    
    for (long long int i = 0; i < conections_count; i++) {
        
        //Use scanf ao invés de cin para ler a entrada! Você está lendo 5x10^6 números!
        //cin  >> vila1 >> vila2 >> year >> hour >> cost;
        scanf("%lld %lld %lld %lld %lld", &vila1, &vila2, &year, &hour, &cost);
        aux1 = {year, hour, cost};
        aux2 = {year, hour, cost};

        //a para b, b para a
        graph[{vila1-1,vila2-1}] = aux1;
        graph[{vila2-1,vila1-1}] = aux2;
        
        //Adj[vila1-1].push_back(vila2-1);
        ADJ_NEW[vila1-1].push_back(vila2-1);
        //Adj[vila2-1].push_back(vila1-1);
        ADJ_NEW[vila2-1].push_back(vila1-1);
        
        //graph[i].push_back(aux1);
        //graph[i].push_back(aux2);
    }
    //fclose(inputFile);
    //testando temp o  de execuçao
    //start = clock()
    //end = double(
    dijkstra(graph, vilas_count, first_year, ADJ_NEW);
    kruskalMST(vilas_count, graph);
    kruskalMST_min_cost(vilas_count, graph);

    return 0;
}
