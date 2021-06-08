/**
 * @file backtrack.cc
 *
 */

#include "backtrack.h"
#include <queue>
#include <ctime>

Backtrack::Backtrack() {}
Backtrack::~Backtrack() {}

size_t numbers = 1;
clock_t start, finish;

void PrintAnswer(const CandidateSet &cs, std::vector<Vertex> &answer) {
  size_t non_zero = 0;
  while (non_zero < answer.size()) {
    if(answer[non_zero] == -1) {
      break;
    }
    non_zero += 1;
  };

  /* answer에서 첫 번쨰로 비어있는 vertex의 위치를 저장한 non_zero 변수가 answer.size()와 같지 않으면
     answer[non_zero]를 candidateset에서 하나씩 대입하고 다시 PrintAnswer을 호출한다. */
  if(non_zero != answer.size()) {
    for(size_t i = 0; i < cs.GetCandidateSize((Vertex)non_zero); ++i) {
      answer[non_zero] = cs.GetCandidate((Vertex)non_zero, i);
      bool checkExist = true;
      for (size_t k = 0; k < answer.size(); ++k) {
        if(k != non_zero) {
          if(answer[k] == answer[non_zero]) {
            checkExist = false;
            break;
          }
        }
      }

      if(checkExist) {
        PrintAnswer(cs, answer);
      }
      answer[non_zero] = -1;
    }
  }

  // non_zero == answer.size()인 경우 정답이기 때문에 출력을 진행한다.
  else {
    std::cout << "a";
    for(size_t i = 0; i < answer.size(); ++i) {
      std::cout << " " << answer[i];
    }
    std::cout << "\n";
    numbers = numbers + 1;
    if (numbers == 100001) {
      exit(0);
    }
  }
}

void DFS(const Graph &data, const CandidateSet &cs, const std::vector<std::pair<Vertex, Vertex>> &intersect,
         std::vector<Vertex> &answer, size_t idx, size_t total) {
  // idx = total인 경우 모든 edge들을 체크하였기 때문에 정답칸에서 비어있는 칸을 채우는 함수인 PrintAnswer로 이동한다.
  if(idx == total) {
    PrintAnswer(cs, answer);
    return;
  }

  // current에는 현재 체크할 edge인 <Vertex, Vertex>를 저장한다.
  const std::pair<Vertex, Vertex> current = intersect[idx];

  /* current의 vertex에 해당하는 answer이 비어있는 경우 candidateset에서 채워넣으면서 neighbor을 체크하고,
     비어있지 않은 경우는 neighbor만 체크한다.
     neighbor 체크가 성공할 경우 다음 edge를 비교하기 위해 DFS를 새로 호출한다.
     이 과정에서 대입할 vertex가 중복이 될 경우를 고려하여 현재 answer에 대입할 vertex가
     기존에 있는지 체크하고 그렇지 않은 경우에만 neighbor인지 체크한다.
  */
  if (answer[current.first] == -1) {
    if (answer[current.second] == -1) {
      for (size_t i = 0; i < cs.GetCandidateSize(current.first); ++i) {
        for(size_t j = 0; j < cs.GetCandidateSize(current.second); ++j) {

          bool checkExist = true;
          for (size_t k = 0; k < answer.size(); ++k) {
            if (answer[k] == cs.GetCandidate(current.first, i)) {
              checkExist = false;
            }
            if (answer[k] == cs.GetCandidate(current.second, j)) {
              checkExist = false;
            }
          }

          if (checkExist) {
            answer[current.first] = cs.GetCandidate(current.first, i);
            answer[current.second] = cs.GetCandidate(current.second, j);

            if(data.IsNeighbor(answer[current.first], answer[current.second])) {
              DFS(data, cs, intersect, answer, idx + 1, total);
            }

            answer[current.first] = -1;
            answer[current.second] = -1;
          }
        }
      }
    }

    else {
      for(size_t i = 0; i < cs.GetCandidateSize(current.first); ++i) {

        bool checkExist = true;

        for (size_t k = 0; k < answer.size(); ++k) {
          if (answer[k] == cs.GetCandidate(current.first, i)) {
            checkExist = false;
          }
        }

        if (checkExist) {
          answer[current.first] = cs.GetCandidate(current.first, i);

          if(data.IsNeighbor(answer[current.first], answer[current.second])) {
              DFS(data, cs, intersect, answer, idx + 1, total);
          }

          answer[current.first] = -1;
        }
      }
    }
  }

  else {
    if(answer[current.second] == -1) {
      for(size_t j = 0; j < cs.GetCandidateSize(current.second); ++j) {

        bool checkExist = true;
        for (size_t k = 0; k < answer.size(); ++k) {
          if (answer[k] == cs.GetCandidate(current.second, j)) {
            checkExist = false;
          }
        }

        if (checkExist) {
          answer[current.second] = cs.GetCandidate(current.second, j);

          if(data.IsNeighbor(answer[current.first], answer[current.second])) {
            DFS(data, cs, intersect, answer, idx + 1, total);
          }

          answer[current.second] = -1;
        }
      }
    }
    else {
      if(data.IsNeighbor(answer[current.first], answer[current.second])) {
        DFS(data, cs, intersect, answer, idx + 1, total);
      }
    }
  }

  return;
}

void Backtrack::PrintAllMatches(const Graph &data, const Graph &query,
                                const CandidateSet &cs) {

  start = clock();

  std::cout << "t " << query.GetNumVertices() << "\n";

  std::vector<std::pair<Vertex, Vertex>> unique_intersect;
  std::vector<std::pair<Vertex, Vertex>> intersect;
  std::vector<std::pair<double, Vertex>> supply_intersect;
  std::vector<Vertex> answer(query.GetNumVertices(), -1);
  std::queue<size_t> vertex_queue;              // 각 점들을 모두 돌기 위핸 queue이다.
  std::queue<size_t> supply_queue;              // 중간에 share이 1인 점들을 만나 BFS가 중단될 경우를 대비하여 share에 따라 vertex를 임시로 저장하는 queue이다.
  bool vertex_check[answer.size()] = { false }; // queue에서 해당 vertex를 체크하였는지 판별하는 bool 배열이다.
  bool finished[answer.size()] = { false };
  size_t start_vertex_dense;
  size_t start_vertex_sparse;
  double share_dense[answer.size()] = { 1 };    // 각 점의 share_dense를 저장하는 double 배열이다.
  double share_sparse[answer.size()] = { 0 };   // 각 점의 share_sparse를 저장하는 double 배열이다.
  double min_share = 1;
  double max_share = 0;
  double mean_share = 0;

  for(size_t i = 0; i < query.NeighborSize(); ++i) {
    size_t neighbor_candidate_size = 0;
    size_t total_neighbor = 0;
    if(cs.GetCandidateSize(i) == 1) {
      answer[i] = cs.GetCandidate(i, 0);
    }
    // 각 점의 share을 계산하는 과정으로 각 점의 이웃에 대해 실제 candidateset에서 neighbor이 되어있는 정도를 계산한다.
    for(size_t j = 0; j < query.NeighborList(i).size(); ++j) {
      neighbor_candidate_size += cs.GetCandidateSize(query.Neighbor(i, j));
      for(size_t k = 0; k < cs.GetCandidateSize(i); ++k) {
        for(size_t l = 0; l < cs.GetCandidateSize(query.Neighbor(i, j)); ++l) {
          total_neighbor += (data.IsNeighbor(cs.GetCandidate(i, k), cs.GetCandidate(query.Neighbor(i, j), l))) ? 1 : 0;
        }
      }
    }

    // dense는 share이 1에 가까운 vertex부터 내림차순으로 탐색을 진행하기 위해 실제 share을 1에서 빼서 저장한다.
    // sparse는 share이 0에 가까운 vertex부터 오름차순으로 탐색을 진행하기 위해 share을 저장한다.
    double result_share_sparse = (double)(total_neighbor) / (double)(cs.GetCandidateSize(i) * neighbor_candidate_size);
    double result_share_dense = 1 - result_share_sparse;
    if(result_share_dense < min_share && result_share_dense > 0) {
      min_share = result_share_dense;
      start_vertex_dense = i;
    }
    if(result_share_sparse > max_share && result_share_sparse < 1) {
      max_share = result_share_sparse;
      start_vertex_sparse = i;
    }

    /* result_share_sparse가 1인 경우 해당 vertex의 모든 candidateset이 neighbor의 candidateset과 연결되어 있으므로
       해당 vertex는 BFS에서 방문하지 않게끔 한다. */
    if(result_share_dense == 0) {
      vertex_check[i] = true;
    }
    else {
      supply_intersect.push_back(std::make_pair(result_share_sparse, i));
    }
    share_dense[i] = result_share_dense;
    share_sparse[i] = result_share_sparse;
    mean_share += result_share_sparse;
  }

  mean_share /= query.GetNumVertices();

  // mean_share이 0.85보다 클 경우에는 sparse한 vertex부터 각 vertex의 neighbor을 intersect에 push한다.
  if(mean_share >= 0.85) {
    vertex_queue.push(start_vertex_sparse);
    vertex_check[start_vertex_sparse] = true;
    std::sort(supply_intersect.begin(), supply_intersect.end());
    for(size_t i = 0; i < supply_intersect.size(); ++i) {
      supply_queue.push(supply_intersect[i].second);
    }
  }

  // mean_share이 0.85보다 작은 경우에는 dense한 vertex부터 각 vertex의 neighbor을 intersect에 push한다.
  else {
    vertex_queue.push(start_vertex_dense);
    vertex_check[start_vertex_dense] = true;
    for(size_t i = 0; i < supply_intersect.size(); ++i) {
      supply_intersect[i].first = 1 - supply_intersect[i].first;
    }
    std::sort(supply_intersect.begin(), supply_intersect.end());
    for(size_t i = 0; i < supply_intersect.size(); ++i) {
      supply_queue.push(supply_intersect[i].second);
    }
  }

  vertex_queue.push(supply_queue.front());
  supply_queue.pop();

  while(!vertex_queue.empty()) {
    size_t vertex = vertex_queue.front();
    std::vector<std::pair<double, size_t>> temp;
    for(size_t i = 0; i < query.NeighborList(vertex).size(); ++i) {
      if(!finished[query.Neighbor(vertex, i)] && share_sparse[query.Neighbor(vertex, i)] > 0) {
        // sparse dense를 구분하는 기준인 0.85보다 크거나 작음에 따라서 정렬 방법을 결정한다.
        if(mean_share >= 0.85) {
          temp.push_back(std::make_pair(share_sparse[query.Neighbor(vertex, i)], query.Neighbor(vertex, i)));
        }
        else {
          temp.push_back(std::make_pair(share_dense[query.Neighbor(vertex, i)], query.Neighbor(vertex, i)));
        }
      }
    }
    std::sort(temp.begin(), temp.end());
    for(size_t i = 0; i < temp.size(); ++i) {
      intersect.push_back(std::make_pair(vertex, temp[i].second));
      if(!vertex_check[temp[i].second]) {
        vertex_queue.push(temp[i].second);
        vertex_check[temp[i].second] = true;
      }
    }
    finished[vertex] = true;
    vertex_queue.pop();
    /* 시작 vertex로부터 share이 1이 아닌 모든 vertex를 BFS하였지만 모든 vertex를 탐색한 상태는 아닐 경우
       임시로 vertex를 저장한 queue인 supply_queue가 비어있지 않으면 vertex를 pop한다. */
    if(vertex_queue.empty() && !supply_queue.empty()) {
      size_t temp = supply_queue.front();
      vertex_queue.push(temp);
      supply_queue.pop();
    }
  }

  // mean_share이 0.85보다 크거나 같을 경우 시작 vertex를 start_vertex_sparse로 정하고 DFS를 진행한다.
  if(mean_share >= 0.85) {
    for(size_t i = 0; i < cs.GetCandidateSize(start_vertex_sparse); ++i) {
      answer[start_vertex_sparse] = cs.GetCandidate(start_vertex_sparse, i);
      DFS(data, cs, intersect, answer, 0, intersect.size());
      answer[start_vertex_sparse] = -1;
    }
  }

  // mean_share이 0.85보다 작을 경우 시작 vertex를 start_vertex_dense로 정하고 DFS를 진행한다.
  else {
    for(size_t i = 0; i < cs.GetCandidateSize(start_vertex_dense); ++i) {
      answer[start_vertex_dense] = cs.GetCandidate(start_vertex_dense, i);
      DFS(data, cs, intersect, answer, 0, intersect.size());
      answer[start_vertex_dense] = -1;
    }
  }

  finish = clock();
}
