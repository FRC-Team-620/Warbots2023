#include <iostream>
#include <string>
#include <cstring>

#define MAX_MSG_LENGTH 300
#define OPTION_NEW_BRANCH "-b"

// base case
constexpr char concat_cstr() {
    return '\0';
}

// concatenate C strings (char*)
template<typename Base, typename... Rest>
std::string concat_cstr(const Base& first, const Rest&... rest) {
    return std::string(first) + concat_cstr(rest...);
}

bool consents(char c) {
    return c == 'y' || c == 'Y';
}

int main(int argc, char** argv) {

    if(argc < 2) {
        std::cout << "Too few arguments.\n";
        return 0;
    }    

    system("git status");
    // std::cout << "git status\n";
    
    bool new_branch = argc > 2 && strcmp(argv[1], OPTION_NEW_BRANCH) == 0; //std::string(argv[1]) == OPTION_NEW_BRANCH;
    std::string push_to = new_branch ? argv[2] : argv[1];

    if(new_branch) {

        std::cout << "\nAre you sure you want to create and checkout a new branch '" + push_to + "'?\n";
        std::cout << "[y/n]: ";

        char should_branch;
        std::cin >> should_branch;
        if(!consents(should_branch))
            return 0;

        system(concat_cstr("git branch ", push_to).c_str());
        system(concat_cstr("git checkout ", push_to).c_str());
        // std::cout << "SYSTEM\t" << concat_cstr("git branch ", push_to).c_str() << "\n";
        // std::cout << "SYSTEM\t" << concat_cstr("git checkout ", push_to).c_str() << "\n";
    }   

    std::cout << "\nAre you sure you want to push all changes to '" << push_to << "'?\n";
    std::cout << "[y/n]: ";
    
    char should_push;
    std::cin >> should_push;
    if(!consents(should_push))
        return 0;

    std::cout << "Please enter a commit message (max " << MAX_MSG_LENGTH << " chars).\n";
    std::cout << "Message: ";

    std::cin.get(); // waits for enter key

    char buffer[MAX_MSG_LENGTH];
    std::cin.getline(buffer, MAX_MSG_LENGTH);
    
    system("git add .");
    system(concat_cstr("git commit -m \"", buffer, "\"").c_str());
    std::cout << "\n";
    system(concat_cstr("git push origin ", push_to).c_str());
    // std::cout << "SYSTEM\t" << "git add .\n";
    // std::cout << "SYSTEM\t" << concat_cstr("git commit -m \"", buffer, "\"").c_str() << "\n";
    // std::cout << "SYSTEM\t" << concat_cstr("git push origin ", push_to).c_str() << "\n";

    return 0;
}