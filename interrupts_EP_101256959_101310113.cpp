/**
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */

#include <interrupts_101256959_101310113.hpp>

void FCFS(std::vector<PCB> &ready_queue) {
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time > second.arrival_time); 
                } 
            );
}

//// ===== CHANGE START: Implemented EP scheduler run_simulation (no preemption) =====
std::tuple<std::string> run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue; // processes that are ready
    std::vector<PCB> job_list; // all processes that have been created/seen
    std::vector<std::pair<int, unsigned int>> io_list; // (PID, io_complete_time)

    unsigned int current_time = 0;
    PCB running;
    idle_CPU(running);

    std::string execution_status = print_exec_header();

    // Helper function to find index in job_list by PID
    auto find_job_idx = [&](int pid)->int {
        for (size_t i = 0; i < job_list.size(); ++i) {
            if (job_list[i].PID == pid) return (int)i;
        }
        return -1;
    };

    // Helper function to build memory snapshot string
    auto memory_snapshot = [&]()->std::string {
        std::stringstream ss;
        unsigned int used = 0, total = 0, usable = 0;
        for (int i = 0; i < 6; ++i) {
            total += memory_paritions[i].size;
            if (memory_paritions[i].occupied != -1) used += memory_paritions[i].size;
            else usable += memory_paritions[i].size;
            ss << "[" << memory_paritions[i].partition_number << ":" << memory_paritions[i].size
               << "MB:" << (memory_paritions[i].occupied == -1 ? "free" : std::to_string(memory_paritions[i].occupied)) << "] ";
        }
        ss << "\nUsed: " << used << " MB / " << total << " MB. Usable free: " << usable << " MB\n";
        return ss.str();
    };

    while (true) {

        // Arrivals
        for (auto &p : list_processes) {
            if (p.arrival_time == current_time) {
                // create a job_list entry (we copy p so we can modify fields like partition & state)
                PCB newproc = p;
                execution_status += print_exec_status(current_time, newproc.PID, NEW, NEW);

                // try to allocate memory
                bool allocated = assign_memory(newproc);

                // record to job_list
                job_list.push_back(newproc);
                int idx = (int)job_list.size() - 1;

                if (allocated) {
                    // ready immediately
                    job_list[idx].state = READY;
                    // set next_io_time if not already set (header add_process should have set next_io_time)
                    if (job_list[idx].next_io_time == 0) job_list[idx].next_io_time = (job_list[idx].io_freq > 0 ? job_list[idx].io_freq : NO_IO);
                    ready_queue.push_back(job_list[idx]);
                    execution_status += print_exec_status(current_time, job_list[idx].PID, NEW, READY);
                    execution_status += memory_snapshot();
                } else {
                    job_list[idx].state = NEW; // waiting for memory
                    execution_status += std::string("    No memory available for PID ") + std::to_string(job_list[idx].PID) + "\n";
                }
            }
        }

        // I/O completions
        for (auto it = io_list.begin(); it != io_list.end();) {
            if (it->second == current_time) {
                int pid = it->first;
                int jidx = find_job_idx(pid);
                if (jidx != -1) {
                    execution_status += print_exec_status(current_time, job_list[jidx].PID, WAITING, READY);
                    job_list[jidx].state = READY;
                    job_list[jidx].next_io_time = (job_list[jidx].io_freq > 0 ? current_time + job_list[jidx].io_freq : NO_IO);
                    ready_queue.push_back(job_list[jidx]);
                }
                it = io_list.erase(it);
            } else ++it;
        }

        // If CPU is idle, dispatch highest-priority READY (no preemption)
        if (running.state != RUNNING) {
            if (!ready_queue.empty()) {
                // sort by priority (lower number = higher priority); tie-breaker arrival_time
                std::sort(ready_queue.begin(), ready_queue.end(), [](const PCB &a, const PCB &b){
                    if (a.priority != b.priority) return a.priority < b.priority;
                    return a.arrival_time < b.arrival_time;
                });

                // dispatch first
                PCB sel = ready_queue.front();
                ready_queue.erase(ready_queue.begin());
                execution_status += print_exec_status(current_time, sel.PID, READY, RUNNING);
                sel.state = RUNNING;
                // set start_time if unset
                if (sel.start_time == -1) sel.start_time = (int)current_time;
                running = sel;

                // update job_list copy to RUNNING
                int jidx = find_job_idx(running.PID);
                if (jidx != -1) {
                    job_list[jidx].state = RUNNING;
                    job_list[jidx].start_time = running.start_time;
                }
            }
        }

        // Advance CPU (1 ms step)
        if (running.state == RUNNING && running.PID != -1) {
            // consume 1 ms
            if (running.remaining_time > 0) running.remaining_time -= 1;
            // sync back to job_list
            int jidx = find_job_idx(running.PID);
            if (jidx != -1) job_list[jidx].remaining_time = running.remaining_time;

            // check I/O request
            if (running.io_freq > 0 && running.next_io_time != NO_IO && current_time + 1 == running.next_io_time) {
                // running -> WAITING
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, WAITING);
                // update job_list, schedule I/O completion
                if (jidx != -1) {
                    job_list[jidx].state = WAITING;
                    job_list[jidx].next_io_time = NO_IO;
                    io_list.emplace_back(job_list[jidx].PID, current_time + 1 + job_list[jidx].io_duration);
                }
                idle_CPU(running);
            } else if (running.remaining_time == 0) {
                // finished
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, TERMINATED);
                if (jidx != -1) {
                    job_list[jidx].state = TERMINATED;
                    free_memory(job_list[jidx]);
                }
                // log memory status
                execution_status += memory_snapshot();
                idle_CPU(running);
            }
        }

        // Termination condition: no future arrivals AND all seen jobs terminated
        bool all_arrived = true;
        for (auto &p : list_processes) if (p.arrival_time > current_time) { all_arrived = false; break; }
        if (all_arrived && all_process_terminated(job_list)) break;

        current_time += 1;
    }

    execution_status += print_exec_footer();
    return std::make_tuple(execution_status);
}

int main(int argc, char** argv) {

    // Get the input file from the user
    if(argc != 2) {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrutps <your_input_file.txt>" << std::endl;
        return -1;
    }

    // Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    // Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    // Parse the entire input file and populate a vector of PCBs.
    // To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while(std::getline(input_file, line)) {
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    // With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    write_output(exec, "execution.txt");

    return 0;
}
