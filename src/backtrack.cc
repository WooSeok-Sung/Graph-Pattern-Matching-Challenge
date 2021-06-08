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

  /* answer���� ù ������ ����ִ� vertex�� ��ġ�� ������ non_zero ������ answer.size()�� ���� ������
     answer[non_zero]�� candidateset���� �ϳ��� �����ϰ� �ٽ� PrintAnswer�� ȣ���Ѵ�. */
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

  // non_zero == answer.size()�� ��� �����̱� ������ ����� �����Ѵ�.
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
  // idx = total�� ��� ��� edge���� üũ�Ͽ��� ������ ����ĭ���� ����ִ� ĭ�� ä��� �Լ��� PrintAnswer�� �̵��Ѵ�.
  if(idx == total) {
    PrintAnswer(cs, answer);
    return;
  }

  // current���� ���� üũ�� edge�� <Vertex, Vertex>�� �����Ѵ�.
  const std::pair<Vertex, Vertex> current = intersect[idx];

  /* current�� vertex�� �ش��ϴ� answer�� ����ִ� ��� candidateset���� ä�������鼭 neighbor�� üũ�ϰ�,
     ������� ���� ���� neighbor�� üũ�Ѵ�.
     neighbor üũ�� ������ ��� ���� edge�� ���ϱ� ���� DFS�� ���� ȣ���Ѵ�.
     �� �������� ������ vertex�� �ߺ��� �� ��츦 ����Ͽ� ���� answer�� ������ vertex��
     ������ �ִ��� üũ�ϰ� �׷��� ���� ��쿡�� neighbor���� üũ�Ѵ�.
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
  std::queue<size_t> vertex_queue;              // �� ������ ��� ���� ���� queue�̴�.
  std::queue<size_t> supply_queue;              // �߰��� share�� 1�� ������ ���� BFS�� �ߴܵ� ��츦 ����Ͽ� share�� ���� vertex�� �ӽ÷� �����ϴ� queue�̴�.
  bool vertex_check[answer.size()] = { false }; // queue���� �ش� vertex�� üũ�Ͽ����� �Ǻ��ϴ� bool �迭�̴�.
  bool finished[answer.size()] = { false };
  size_t start_vertex_dense;
  size_t start_vertex_sparse;
  double share_dense[answer.size()] = { 1 };    // �� ���� share_dense�� �����ϴ� double �迭�̴�.
  double share_sparse[answer.size()] = { 0 };   // �� ���� share_sparse�� �����ϴ� double �迭�̴�.
  double min_share = 1;
  double max_share = 0;
  double mean_share = 0;

  for(size_t i = 0; i < query.NeighborSize(); ++i) {
    size_t neighbor_candidate_size = 0;
    size_t total_neighbor = 0;
    if(cs.GetCandidateSize(i) == 1) {
      answer[i] = cs.GetCandidate(i, 0);
    }
    // �� ���� share�� ����ϴ� �������� �� ���� �̿��� ���� ���� candidateset���� neighbor�� �Ǿ��ִ� ������ ����Ѵ�.
    for(size_t j = 0; j < query.NeighborList(i).size(); ++j) {
      neighbor_candidate_size += cs.GetCandidateSize(query.Neighbor(i, j));
      for(size_t k = 0; k < cs.GetCandidateSize(i); ++k) {
        for(size_t l = 0; l < cs.GetCandidateSize(query.Neighbor(i, j)); ++l) {
          total_neighbor += (data.IsNeighbor(cs.GetCandidate(i, k), cs.GetCandidate(query.Neighbor(i, j), l))) ? 1 : 0;
        }
      }
    }

    // dense�� share�� 1�� ����� vertex���� ������������ Ž���� �����ϱ� ���� ���� share�� 1���� ���� �����Ѵ�.
    // sparse�� share�� 0�� ����� vertex���� ������������ Ž���� �����ϱ� ���� share�� �����Ѵ�.
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

    /* result_share_sparse�� 1�� ��� �ش� vertex�� ��� candidateset�� neighbor�� candidateset�� ����Ǿ� �����Ƿ�
       �ش� vertex�� BFS���� �湮���� �ʰԲ� �Ѵ�. */
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

  // mean_share�� 0.85���� Ŭ ��쿡�� sparse�� vertex���� �� vertex�� neighbor�� intersect�� push�Ѵ�.
  if(mean_share >= 0.85) {
    vertex_queue.push(start_vertex_sparse);
    vertex_check[start_vertex_sparse] = true;
    std::sort(supply_intersect.begin(), supply_intersect.end());
    for(size_t i = 0; i < supply_intersect.size(); ++i) {
      supply_queue.push(supply_intersect[i].second);
    }
  }

  // mean_share�� 0.85���� ���� ��쿡�� dense�� vertex���� �� vertex�� neighbor�� intersect�� push�Ѵ�.
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
        // sparse dense�� �����ϴ� ������ 0.85���� ũ�ų� ������ ���� ���� ����� �����Ѵ�.
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
    /* ���� vertex�κ��� share�� 1�� �ƴ� ��� vertex�� BFS�Ͽ����� ��� vertex�� Ž���� ���´� �ƴ� ���
       �ӽ÷� vertex�� ������ queue�� supply_queue�� ������� ������ vertex�� pop�Ѵ�. */
    if(vertex_queue.empty() && !supply_queue.empty()) {
      size_t temp = supply_queue.front();
      vertex_queue.push(temp);
      supply_queue.pop();
    }
  }

  // mean_share�� 0.85���� ũ�ų� ���� ��� ���� vertex�� start_vertex_sparse�� ���ϰ� DFS�� �����Ѵ�.
  if(mean_share >= 0.85) {
    for(size_t i = 0; i < cs.GetCandidateSize(start_vertex_sparse); ++i) {
      answer[start_vertex_sparse] = cs.GetCandidate(start_vertex_sparse, i);
      DFS(data, cs, intersect, answer, 0, intersect.size());
      answer[start_vertex_sparse] = -1;
    }
  }

  // mean_share�� 0.85���� ���� ��� ���� vertex�� start_vertex_dense�� ���ϰ� DFS�� �����Ѵ�.
  else {
    for(size_t i = 0; i < cs.GetCandidateSize(start_vertex_dense); ++i) {
      answer[start_vertex_dense] = cs.GetCandidate(start_vertex_dense, i);
      DFS(data, cs, intersect, answer, 0, intersect.size());
      answer[start_vertex_dense] = -1;
    }
  }

  finish = clock();
}
