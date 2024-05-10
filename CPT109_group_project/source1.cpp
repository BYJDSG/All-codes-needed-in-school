#include <iostream>
#include <fstream>
#include <string>
#include <limits> // For numeric_limits
#include <cctype> // for character handling functions
#include <vector>
using namespace std;

class Dish {
private:
    string name;
    int id;
    double price;
    string ingredients;
    double cost;

public:
    Dish(string dish_name, int dish_id, double dish_price, string dish_ingredients, double dish_cost)
        : name(dish_name), id(dish_id), price(dish_price), ingredients(dish_ingredients), cost(dish_cost) {}

    void getDishInfo() const {
        cout << "Dish Name: " << name << endl;
        cout << "Dish ID: " << id << endl;
        cout << "Price: " << price << endl;
        cout << "Ingredients: " << ingredients << endl;
        cout << "Cost: " << cost << endl;
    }

    void writeToFile(ofstream& file) const {
        file << "Dish Name: " << name << endl;
        file << "Dish ID: " << id << endl;
        file << "Price: " << price << endl;
        file << "Ingredients: " << ingredients << endl;
        file << "Cost: " << cost << endl;
        file << endl;
    }

    string getName() const {
        return name;
    }

    int getId() const {
        return id;
    }
};

class User {
protected:
    string username;
    string email;
    string userinfo_filename = "userinfo.txt";
    int age;

public:
    User(string uname, string mail, int user_age) {
        username = uname;
        email = mail;
        age = user_age;
        
    }
    virtual ~User() {}

    virtual void getUserInfo() {
        cout << "Username: " << username << endl;
        cout << "Email: " << email << endl;
        cout << "Age: " << age << endl;
    }

    virtual void editUserInfo() {
        cout << "Enter new username: ";
        cin >> username;
        cout << "Enter new email: ";
        cin >> email;
        cout << "Enter new age: ";
        cin >> age;
    }

    virtual string getPassword() const {
        return ""; // 默认返回空字符串
    }

    // 将用户信息写入文件
    virtual void writeToFile(ofstream& file) = 0;

    // 检查用户信息是否存在于文件中   //已修改
    bool userInfoExists(const string& filename) {
        ifstream infile(filename);
        if (!infile) {
            cerr << "Error opening file." << endl;
            return false;
        }

        string line;
        string usernameToFind = "Email: " + email;
        while (getline(infile, line)) {
            if (line.find(usernameToFind) != string::npos) {
                infile.close();
                return true; // 用户信息已存在
            }
        }

        infile.close();
        return false; // 用户信息不存在
    }
};

class Manager : public User {
private:
    string password; // 添加密码属性

public:
    Manager(string uname, string pwd, string mail, int user_age) : User(uname, mail, user_age), password(pwd) {}

    void getUserInfo() override {
        cout << "Manager Information:" << endl;
        User::getUserInfo();
    }

    void editUserInfo() override {
        cout << "Editing Manager Information:" << endl;
        User::editUserInfo();
    }

    string getPassword() const override {
        return password;
    }
    void writeToFile(ofstream& userinfo_filename) {
        userinfo_filename << "Username: " << username << endl;
        userinfo_filename << "Email: " << email << endl;
        userinfo_filename << "Age: " << age << endl;
        userinfo_filename << "Password: " << password << endl;
        userinfo_filename << endl;
    }
    
    void print_dish_menu(ifstream& dish_file) {

    }
};

class Chef : public User {
private:
    string password; // 添加密码属性

public:
    Chef(string uname, string pwd, string mail, int user_age) : User(uname, mail, user_age), password(pwd) {}

    void getUserInfo() override {
        cout << "Chef Information:" << endl;
        User::getUserInfo();
    }

    void editUserInfo() override {
        cout << "Editing Chef Information:" << endl;
        User::editUserInfo();
    }

    string getPassword() const override {
        return password;
    }
    void writeToFile(ofstream& userinfo_filename) {
        userinfo_filename << "Username: " << username << endl;
        userinfo_filename << "Email: " << email << endl;
        userinfo_filename << "Age: " << age << endl;
        userinfo_filename << "Password: " << password << endl;
        userinfo_filename << endl;
    }
    void addNewDish(int dish_id) {
        cout << "\nAdding New Dish:" << endl;
        string dish_name, dish_ingredients;
        double dish_price, dish_cost;

        cout << "Enter Dish Name: ";
        cin >> dish_name;
        cout << "Enter Price: ";
        cin >> dish_price;
        cout << "Enter Ingredients: ";
        cin.ignore(); // 忽略之前的换行符
        getline(cin, dish_ingredients);
        cout << "Enter Cost: ";
        cin >> dish_cost;

        vector<Dish> dishes = readDishesFromFile("Dish.txt");
        for (const auto& dish : dishes) {
            if (dish.getId() == dish_id) {
                cout << "Dish with the same ID already exists. Not adding." << endl;
                return;
            }
        }

        ofstream dishFile("Dish.txt", ios::app); // 打开文件以追加方式写入
        if (!dishFile.is_open()) {
            cout << "Error opening file!" << endl;
            return;
        }

        Dish newDish(dish_name, dish_id, dish_price, dish_ingredients, dish_cost);
        newDish.writeToFile(dishFile); // 将菜品信息写入文件
        cout << "New Dish Added Successfully!" << endl;

        dishFile.close(); // 关闭文件
    }

    vector<Dish> readDishesFromFile(const string& filename) const {
        vector<Dish> dishes;
        ifstream infile(filename);
        if (!infile) {
            cerr << "Error opening file." << endl;
            return dishes;
        }

        string line;
        string dish_name;
        int dish_id;
        double dish_price;
        string dish_ingredients;
        double dish_cost;
        while (getline(infile, line)) {
            if (line.find("Dish Name: ") != string::npos) {
                dish_name = line.substr(11);
                getline(infile, line);
                dish_id = stoi(line.substr(9));
                getline(infile, line);
                dish_price = stod(line.substr(7));
                getline(infile, line);
                dish_ingredients = line.substr(14);
                getline(infile, line);
                dish_cost = stod(line.substr(6));

                Dish dish(dish_name, dish_id, dish_price, dish_ingredients, dish_cost);
                dishes.push_back(dish);
            }
        }

        infile.close();
        return dishes;
    }

    void deleteDish(int dish_id) {
        vector<Dish> dishes = readDishesFromFile("Dish.txt");
        bool found = false;
        ofstream tempFile("temp.txt");
        for (const auto& dish : dishes) {
            if (dish.getId() == dish_id) {
                found = true;
                cout << "Dish deleted successfully:" << endl;
            }
            else {
                dish.writeToFile(tempFile); // 将非被删除的菜品写入临时文件
            }
        }
        tempFile.close();

        if (!found) {
            cout << "Dish not found!" << endl;
            remove("temp.txt"); // 删除临时文件
            return;
        }

        // 删除原文件并重命名临时文件
        remove("Dish.txt");
        rename("temp.txt", "Dish.txt");
    }
};

class Customer : public User {
public:
    Customer(string uname, string mail, int user_age) : User(uname, mail, user_age) {
        if(!userInfoExists(userinfo_filename)) {
            ofstream ofile(userinfo_filename,ios_base::app);
            if (!ofile) {
                cerr << "Error opening file." << endl;
            }else{
                writeToFile(ofile);
                customer_operating_UI();
            }
        }
    }

    void getUserInfo() override {
        cout << "Customer Information:" << endl;
        User::getUserInfo();
    }

    void editUserInfo() override {
        cout << "Editing Customer Information:" << endl;
        User::editUserInfo();
    }
    void writeToFile(ofstream& userinfo_filename) override {
        userinfo_filename << "Username: " << username << endl;
        userinfo_filename << "Email: " << email << endl;
        userinfo_filename << "Age: " << age << endl;
        userinfo_filename << endl;
    }

    void customer_operating_UI(){
        print_dish_menu();
    }
};

// bool authenticate(User* user) {
//     // 只有Manager和Chef需要密码验证
//     if (dynamic_cast<Manager*>(user) || dynamic_cast<Chef*>(user)) {
//         string correctPassword = user->getPassword();
//         string inputPassword;
//         int attempts = 0;
//         while (attempts < 3) {
//             cout << "Enter your password: ";
//             cin >> inputPassword;
//             if (inputPassword == correctPassword) {
//                 return true;
//             }
//             else {
//                 attempts++;
//                 cout << "Incorrect password. Please try again." << endl;
//             }
//         }
//         cout << "Too many incorrect attempts. Exiting..." << endl;
//         return false;
//     }
//     return true; // Customer不需要密码验证
// }

// void printMenu() {
//     cout << "\nMenu:" << endl;
//     cout << "1. Edit User Info" << endl;
//     cout << "2. Exit" << endl;
//     cout << "Enter your choice: ";
// }
void customer_logging(User* user){
        string input_name;
        string customor_email;
        int age = 0;
        cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard bad input
        while (true) {
            cout << "Please enter your name: ";
            getline(cin, input_name);

            // Check if the name is empty
            if (input_name.empty()) {
                cout << "Please enter a valid name.\n";
                continue; // Prompt the user to enter a name again
            }

            // Check if the name contains only alphabets and spaces
            bool valid = true;
            for (char c : input_name) {
                if (!isalpha(c) && !isspace(c)) {
                    valid = false;
                    break;
                }
            }

            if (!valid) {
                cout << "Name should contain only alphabets and spaces.\n";
                continue; // Prompt the user to enter a name again
            }

            // If the name is valid, break the loop
            break;
        }
        cout << "Enter Your Email: ";
        while(!(cin >> customor_email) || !(customor_email.find('@'))) {
            cin.clear(); // Clear the error flag
            cerr << "Invalid input. Please enter a right email." << endl;
            cout << "Enter Your Email: ";
            cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard bad input
        }
        cout << "Enter Your age: ";
        while(!(cin >> age) || age <= 0) {
            cin.clear(); // Clear the error flag
            cerr << "Invalid input. Please enter a right age." << endl;
            cout << "Enter Your age: ";
            cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard bad input
        }
        user = new Customer(input_name, customor_email, age); // from here start
        system("cls");
}

int main() {
    int choice;
    cout << "Select your role:" << endl;
    cout << "1. Manager" << endl;
    cout << "2. Chef" << endl;
    cout << "3. Customer" << endl;
    cout << "4. register an account" << endl;
    cout << "Enter your choice: ";
    cin >> choice;

    User* user = nullptr;

    switch (choice) {
    case 1:
        manager_logging(user);
        // user = new Manager("manager123", "123456", "manager@example.com", 35);
        break;
    case 2:
        chef_logging(user);
        // user = new Chef("chef123", "654321", "chef@example.com", 30);
        break;
    case 3:{
        customer_logging(user);
        break;
    }
    case 4:{
        register_operatingUI(user);
        break;
    }
    default:
        cout << "Invalid choice. Exiting..." << endl;
        return 0;
    }

    // if (authenticate(user)) {
    //     cout << "Access granted." << endl;
    //     user->editUserInfo(); // 用户输入个人信息
    //     cout << "Your User Info:" << endl;
    //     user->getUserInfo(); // 打印用户信息

    //     // 检查用户信息是否已存在
    //     if (!user->userInfoExists("Person_Inf.txt")) {
    //         ofstream outFile("Person_Inf.txt", ios::app);
    //         if (!outFile) {
    //             cerr << "Error opening file." << endl;
    //             delete user;
    //             return 1;
    //         }
    //         user->writeToFile(outFile); // 将用户信息写入文件
    //         outFile.close();
    //         cout << "User Info added to file." << endl;
    //     }
    //     else {
    //         cout << "User Info already exists in file." << endl;
    //     }

    //     int menuChoice;
    //     do {
    //         printMenu();
    //         cin >> menuChoice;
    //         switch (menuChoice) {
    //         case 1:
    //             user->editUserInfo(); // 修改用户信息
    //             cout << "Updated User Info:" << endl;
    //             user->getUserInfo(); // 打印更新后的用户信息
    //             break;
    //         case 2:
    //             cout << "Exiting program..." << endl;
    //             break;
    //         default:
    //             cout << "Invalid choice. Please try again." << endl;
    //         }
    //     } while (menuChoice != 2);
    // }
    // else {
    //     cout << "Access denied." << endl;
    // }

    // delete user; // 释放内存

    return 0;
}
