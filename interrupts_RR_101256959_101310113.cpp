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

//// ===== CHANGE START: Implemented RR scheduler run_simulation (quantum = 100 ms) =====
const unsigned int RR_QUANTUM = 100; 

std::tuple<std::string> run_simulation(std::vector<PCB> list_processes) {
    std::vector<PCB> ready_queue; // ready processes
    std::vector<PCB> job_list; // all processes that have been created/seen
    std::vector<std::pair<int, unsigned int>> io_list; // (PID, io_complete_time)

    unsigned int current_time = 0;
    PCB running;
    idle_CPU(running);

    std::string execution_status = print_exec_header();

    auto find_job_idx = [&](int pid)->int {
        for (size_t i = 0; i < job_list.size(); ++i) {
            if (job_list[i].PID == pid) return (int)i;
        }
        return -1;
    };

    unsigned int running_quantum_used = 0;

    while (true) {
        // Arrivals
        for (auto &p : list_processes) {
            if (p.arrival_time == current_time) {
                PCB newproc = p;
                execution_status += print_exec_status(current_time, newproc.PID, NEW, NEW);

                bool allocated = assign_memory(newproc);
                job_list.push_back(newproc);
                int jidx = (int)job_list.size() - 1;

                if (allocated) {
                    job_list[jidx].state = READY;
                    // ensure next_io_time is set; add_process already assigned next_io_time using NO_IO
                    if (job_list[jidx].next_io_time == 0) job_list[jidx].next_io_time = (job_list[jidx].io_freq > 0 ? job_list[jidx].io_freq : NO_IO);
                    ready_queue.push_back(job_list[jidx]);
                    execution_status += print_exec_status(current_time, job_list[jidx].PID, NEW, READY);
                } else {
                    job_list[jidx].state = NEW;
                    execution_status += std::string("    No memory available for PID ") + std::to_string(job_list[jidx].PID) + "\n";
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
                    // push to tail (RR)
                    ready_queue.push_back(job_list[jidx]);
                }
                it = io_list.erase(it);
            } else ++it;
        }

        // Dispatch if CPU idle
        if (running.state != RUNNING) {
            if (!ready_queue.empty()) {
                PCB sel = ready_queue.front();
                ready_queue.erase(ready_queue.begin());
                execution_status += print_exec_status(current_time, sel.PID, READY, RUNNING);
                sel.state = RUNNING;
                sel.time_in_current_quantum = 0;
                if (sel.start_time == -1) sel.start_time = (int)current_time;
                running = sel;
                // update job_list entry
                int jidx = find_job_idx(running.PID);
                if (jidx != -1) job_list[jidx].state = RUNNING;
                running_quantum_used = 0;
            }
        }

        // Run one ms
        if (running.state == RUNNING && running.PID != -1) {
            if (running.remaining_time > 0) running.remaining_time -= 1;
            running.time_in_current_quantum++;
            running_quantum_used++;

            // sync to job_list
            int jidx = find_job_idx(running.PID);
            if (jidx != -1) {
                job_list[jidx].remaining_time = running.remaining_time;
                job_list[jidx].time_in_current_quantum = running.time_in_current_quantum;
            }

            // I/O start
            if (running.io_freq > 0 && running.next_io_time != NO_IO && current_time + 1 == running.next_io_time) {
                // RUNNING -> WAITING
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, WAITING);
                if (jidx != -1) {
                    job_list[jidx].state = WAITING;
                    job_list[jidx].next_io_time = NO_IO;
                    io_list.emplace_back(job_list[jidx].PID, current_time + 1 + job_list[jidx].io_duration);
                }
                idle_CPU(running);
                running_quantum_used = 0;
            } else if (running.remaining_time == 0) {
                // RUNNING -> TERMINATED
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, TERMINATED);
                if (jidx != -1) {
                    job_list[jidx].state = TERMINATED;
                    free_memory(job_list[jidx]);
                }
                idle_CPU(running);
                running_quantum_used = 0;
            } else if (running.time_in_current_quantum >= RR_QUANTUM) {
                // quantum expired -> preempt and push back to ready tail
                execution_status += print_exec_status(current_time + 1, running.PID, RUNNING, READY);
                if (jidx != -1) {
                    job_list[jidx].state = READY;
                    job_list[jidx].time_in_current_quantum = 0;
                    ready_queue.push_back(job_list[jidx]);
                }
                idle_CPU(running);
                running_quantum_used = 0;
            }
        }

        // termination check
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
