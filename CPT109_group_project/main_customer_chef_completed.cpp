#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <string>
#include <limits> // For numeric_limits
#include <cctype> // for character handling functions
#include <vector>
#include <ctime>

using namespace std;

class Order
{
private:
    int order_id;
    string email;
    int year, month, day;
    string time;
    int dishID;
    double price, cost;
    bool paid;

public:
    // 构造函数
    Order(int orderid, const string& email, int year, int month, int day, const string& time,
        int dishID, double price, double cost, bool paid)
        : email(email), year(year), month(month), day(day), time(time),
        dishID(dishID), price(price), cost(cost), paid(paid), order_id(orderid) {}

    // 获取器方法
    int getID() const { return order_id; }
    string getEmail() const { return email; }
    int getYear() const { return year; }
    int getMonth() const { return month; }
    int getDay() const { return day; }
    string getTime() const { return time; }
    int getDishID() const { return dishID; }
    double getPrice() const { return price; }
    double getCost() const { return cost; }
    bool isPaid() const { return paid; }

    // 设置器方法
    void setEmail(const string& val) { email = val; }
    void setYear(int val) { year = val; }
    void setMonth(int val) { month = val; }
    void setDay(int val) { day = val; }
    void setTime(const string& val) { time = val; }
    void setDishID(int val) { dishID = val; }
    void setPrice(double val) { price = val; }
    void setCost(double val) { cost = val; }
    void setPaid(bool val) { paid = val; }

    // 显示订单详情
    void displayOrderDetails() const
    {
        cout << "Order Details:" << endl;
        cout << "Order ID: " << order_id << endl;
        cout << "Email: " << email << endl;
        cout << "Date: " << year << "-" << month << "-" << day << endl;
        cout << "Time: " << time << endl;
        cout << "Dish ID: " << dishID << endl;
        cout << "Price: " << price << endl;
        cout << "Cost: " << cost << endl;
        cout << "Paid: " << (paid ? "Yes" : "No") << endl;
        cout << endl;
    }

    // 将订单信息写入文件
    void writeToFile(const string& filename) const
    {
        ofstream file(filename, ios::app);
        if (!file.is_open())
        {
            cerr << "Unable to open the file for writing." << endl;
            return;
        }

        file << "Email: " << email << "\n";
        file << "ID: " << order_id << ";\n";
        file << "Year: " << year << "\n";
        file << "Month: " << month << "\n";
        file << "Day: " << day << "\n";
        file << "Time: " << time << "\n";
        file << "Dish ID: " << dishID << "\n";
        file << "Price: " << price << "\n";
        file << "Cost: " << cost << "\n";
        file << "Paid: " << (paid ? "1" : "0") << "\n\n";

        file.close();
    }

    // 从文件中删除指定ID的订单
    void deleteOrderFromFile(const string& filename)
    {
        vector<string> lines; // 用于保存文件内容
        ifstream file(filename);
        if (!file.is_open())
        {
            cerr << "Unable to open the file for reading." << endl;
            return;
        }

        string line;
        while (getline(file, line))
        {
            if (line.find("ID: " + to_string(order_id) + ";") == string::npos)
            {
                // 如果当前行不包含指定ID的订单信息，则将其保存到容器中
                lines.push_back(line);
            }
            else
            {
                // 如果当前行包含指定ID的订单信息，则跳过当前订单的完整信息
                for (int i = 0; i < 8; ++i)
                {
                    getline(file, line);
                }
            }
        }
        file.close();

        // 将修改后的内容写回到文件中
        ofstream outfile(filename);
        if (!outfile.is_open())
        {
            cerr << "Unable to open the file for writing." << endl;
            return;
        }
        for (const string& l : lines)
        {
            outfile << l << "\n";
        }
        outfile.close();
    }
};

class Dish
{
private:
    string name;
    int id;
    double price;
    string ingredients;
    double cost;

public:
    Dish(string dish_name, int dish_id, double dish_price, string dish_ingredients, double dish_cost)
        : name(dish_name), id(dish_id), price(dish_price), ingredients(dish_ingredients), cost(dish_cost) {}
    double getProfit() const
    {
        return price - cost; // 售价减去成本
    }

    void getDishInfo() const
    {
        cout << "Dish Name: " << name << endl;
        cout << "Dish ID: " << id << endl;
        cout << "Price: " << price << endl;
        cout << "Ingredients: " << ingredients << endl;
        cout << "Cost: " << cost << endl;
    }

    void writeToFile(ofstream& file) const
    {
        file << "Dish Name: " << name << endl;
        file << "Dish ID: " << id << endl;
        file << "Price: " << price << endl;
        file << "Ingredients: " << ingredients << endl;
        file << "Cost: " << cost << endl;
        file << endl;
    }

    string getName() const
    {
        return name;
    }

    int getId() const
    {
        return id;
    }

    double getPrice() const
    {
        return price;
    }

    double getCost() const
    {
        return cost;
    }
};

class User
{
protected:
    string username;
    string email;
    string userinfo_filename = "userinfo.txt";
    int age;

public:
    User(string uname, string mail, int user_age)
    {
        username = uname;
        email = mail;
        age = user_age;
    }
    virtual ~User() {}

    virtual void getUserInfo()
    {
        cout << "Username: " << username << endl;
        cout << "Email: " << email << endl;
        cout << "Age: " << age << endl;
    }

    virtual void editUserInfo()
    {
        cout << "Enter new username: ";
        cin >> username;
        cout << "Enter new email: ";
        cin >> email;
        cout << "Enter new age: ";
        cin >> age;
    }

    virtual string getPassword() const
    {
        return ""; // 默认返回空字符串
    }

    // 将用户信息写入文件
    virtual void writeToFile(ofstream& file) = 0;

    // 检查用户信息是否存在于文件中   //已修改
    bool userInfoExists(const string& filename)
    {
        ifstream infile(filename);
        if (!infile)
        {
            cerr << "Error opening file." << endl;
            return false;
        }

        string line;
        string usernameToFind = "Email: " + email;
        while (getline(infile, line))
        {
            if (line.find(usernameToFind) != string::npos)
            {
                infile.close();
                return true; // 用户信息已存在
            }
        }

        infile.close();
        return false; // 用户信息不存在
    }
    vector<Dish> readDishesFromFile(const string& filename) const
    {
        vector<Dish> dishes;
        ifstream infile(filename);
        if (!infile)
        {
            cerr << "Error opening file." << endl;
            return dishes;
        }

        string line;
        string dish_name;
        int dish_id;
        double dish_price;
        string dish_ingredients;
        double dish_cost;
        while (getline(infile, line))
        {
            if (line.find("Dish Name: ") != string::npos)
            {
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

    void print_dish_menu(const string& filename) const
    {
        vector<Dish> dishes = readDishesFromFile(filename);
        cout << "Dish Menu:" << endl;
        for (const auto& dish : dishes)
        {
            dish.getDishInfo();
            cout << endl;
        }
    }
};

class Manager : public User
{
private:
    string password; // 添加密码属性
    double totalProfit = 0.0;
    double cost;
    double price;

public:
    Manager(string uname, string pwd, string mail, int user_age) : User(uname, mail, user_age), password(pwd) {}

    void getUserInfo() override
    {
        cout << "Manager Information:" << endl;
        User::getUserInfo();
    }

    void editUserInfo() override
    {
        cout << "Editing Manager Information:" << endl;
        User::editUserInfo();
    }

    string getPassword() const override
    {
        return password;
    }
    void writeToFile(ofstream& userinfo_filename)
    {
        userinfo_filename << "Usertype: " << "Manager" << endl;
        userinfo_filename << "Username: " << username << endl;
        userinfo_filename << "Email: " << email << endl;
        userinfo_filename << "Age: " << age << endl;
        userinfo_filename << "Password: " << password << endl;
        userinfo_filename << endl;
    }
    void viewAllOrders()
    {
        // 实现查看所有订单的逻辑
        cout << "Viewing all orders (placeholder function)." << endl;
    }
    void viewChefInfo()
    {
        // 实现查看厨师信息的逻辑
        cout << "Viewing chef information (placeholder function)." << endl;
    }
    void viewCustomerInfo()
    {
        // 实现查看顾客信息的逻辑
        cout << "Viewing customer information (placeholder function)." << endl;
    }

    void modifyChef(int chefId)
    {
        // 实现修改厨师信息的逻辑
        cout << "Modifying chef with ID: " << chefId << " (placeholder function)." << endl;
    }

    void modifyDish(int dishId)
    {
        // 实现修改菜品信息的逻辑
        cout << "Modifying dish with ID: " << dishId << " (placeholder function)." << endl;
    }
    void addNewDish()
    {
        cout << "\nAdding New Dish:" << endl;
        string dish_name, dish_ingredients;
        double dish_price, dish_cost;
        int newDishId;

        cout << "Enter Dish ID for the new dish: ";
        cin >> newDishId;
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
        for (const auto& dish : dishes)
        {
            if (dish.getId() == newDishId)
            {
                cout << "Dish with the same ID already exists. Not adding." << endl;
                return;
            }
        }

        ofstream dishFile("Dish.txt", ios::app); // 打开文件以追加方式写入
        if (!dishFile.is_open())
        {
            cout << "Error opening file!" << endl;
            return;
        }

        Dish newDish(dish_name, newDishId, dish_price, dish_ingredients, dish_cost);
        newDish.writeToFile(dishFile); // 将菜品信息写入文件
        cout << "New Dish Added Successfully!" << endl;

        dishFile.close(); // 关闭文件
    }

    vector<Dish> readDishesFromFile(const string& filename) const
    {
        vector<Dish> dishes;
        ifstream infile(filename);
        if (!infile)
        {
            cerr << "Error opening file." << endl;
            return dishes;
        }

        string line;
        string dish_name;
        int dish_id;
        double dish_price;
        string dish_ingredients;
        double dish_cost;
        while (getline(infile, line))
        {
            if (line.find("Dish Name: ") != string::npos)
            {
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
    // 删除菜品
    void deleteDish(int dish_id)
    {
        vector<Dish> dishes = readDishesFromFile("Dish.txt");
        bool found = false;
        ofstream tempFile("temp.txt");
        for (const auto& dish : dishes)
        {
            if (dish.getId() == dish_id)
            {
                found = true;
                cout << "Dish deleted successfully:" << endl;
            }
            else
            {
                dish.writeToFile(tempFile); // 将非被删除的菜品写入临时文件
            }
        }
        tempFile.close();

        if (!found)
        {
            cout << "Dish not found!" << endl;
            remove("temp.txt"); // 删除临时文件
            return;
        }

        // 删除原文件并重命名临时文件
        remove("Dish.txt");
        rename("temp.txt", "Dish.txt");
    }
    // double calculateRevenue()
    // {
    //     cout << "Total Profit: $" << totalProfit << endl; // 显示总利润
    //     return totalProfit;
    // }

    void calculateRevenue()
    {
        time_t currentTime = time(nullptr);      // 获取当前时间
        tm* localTime = localtime(&currentTime); // 转换为本地时间

        double totalProfit = 0.0;//初始化利润为0

        ifstream file("orders.txt");//打开订单文件
        if (!file.is_open())//未打开检测
        {
            cerr << "Unable to open the file: " << endl;
            return;
        }

        string line, key, value;//用于读取文件
        string timeString;//用于读取文件
        double price, cost;//用于读取文件
        int year, month, day;//用于读取文件

        while (getline(file, line))//循环读取文件直到文件结束
        {
            if (line.empty())//空行跳过处理
                continue;

            size_t colonPos = line.find(':');//字符长度停在冒号位置
            if (colonPos != string::npos)
            {
                key = line.substr(0, colonPos);//用冒号位置分割数据，前面是属性
                value = line.substr(colonPos + 1);//后部分是值
                if (!value.empty() && value.front() == ' ')//值前面有空格删除空格
                {
                    value.erase(0, 1); // 移除前导空格
                }

                if (key == "Year")//读取年份
                    year = stoi(value);
                else if (key == "Month")//月份，以及下面的具体的时间
                    month = stoi(value);
                else if (key == "Day")
                    day = stoi(value);
                else if (key == "Time")
                    timeString = value;
                else if (key == "Price")
                    price = stod(value);
                else if (key == "Cost")
                    cost = stod(value);

                if (file.peek() == '\n' || file.peek() == EOF)
                {
                    tm orderTime = { 0 };//初始化一个时间类型
                    orderTime.tm_year = year - 1900;//减到tm格式方便计算秒数
                    orderTime.tm_mon = month - 1;//减到tm格式方便计算秒数
                    orderTime.tm_mday = day;

                    int hour, minute, second;
                    sscanf(timeString.c_str(), "%d:%d:%d", &hour, &minute, &second);
                    orderTime.tm_hour = hour;
                    orderTime.tm_min = minute;
                    orderTime.tm_sec = second;

                    time_t orderTimestamp = mktime(&orderTime);//转换成合适格式后转成时间戳，用到函数里面可以直接转换秒数
                    if (difftime(currentTime, orderTimestamp) < 86400) // 86400 秒 = 一天
                    {
                        totalProfit += (price - cost);//符合条件的订单，给总利润加上这个利润
                    }
                }
            }
        }
        cout << "Profit in one day : $" << totalProfit << endl; // 显示总利润
        file.close();//关文件
    }

    void sellDish(int dishId, int quantitySold)
    {
        vector<Dish> dishes = readDishesFromFile("Dish.txt");
        bool found = false;
        for (const auto& dish : dishes)
        {
            if (dish.getId() == dishId)
            {
                double profit = dish.getProfit() * quantitySold;
                totalProfit += profit; // 累加利润
                cout << "Sold " << quantitySold << " units of " << dish.getName() << ". Profit: $" << profit << endl;
                found = true;
                break;
            }
        }
        if (!found)
        {
            cout << "Dish ID " << dishId << " not found." << endl;
        }
    }
};
void ChefprintMenu()
{
    cout << "\nMenu:" << endl;
    cout << "1. Edit User Info" << endl;
    cout << "2. Delete Dish" << endl;
    cout << "3. Add New Dish" << endl; // 添加新选项
    cout << "4. Exit" << endl;
    cout << "Enter your choice: ";
}
class Chef : public User
{
private:
    string password; // 添加密码属性

public:
    Chef(string uname, string pwd, string mail, int user_age) : User(uname, mail, user_age), password(pwd) {}

    void getUserInfo() override
    {
        cout << "Chef Information:" << endl;
        User::getUserInfo();
    }

    void editUserInfo() override
    {
        cout << "Editing Chef Information:" << endl;
        User::editUserInfo();
    }

    string getPassword() const override
    {
        return password;
    }
    void writeToFile(ofstream& userinfo_filename)
    {
        userinfo_filename << "Usertype: " << "Chef" << endl;
        userinfo_filename << "Username: " << username << endl;
        userinfo_filename << "Email: " << email << endl;
        userinfo_filename << "Age: " << age << endl;
        userinfo_filename << "Password: " << password << endl;
        userinfo_filename << endl;
    }
    void addNewDish(int dish_id)
    {
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
        for (const auto& dish : dishes)
        {
            if (dish.getId() == dish_id)
            {
                cout << "Dish with the same ID already exists. Not adding." << endl;
                return;
            }
        }

        ofstream dishFile("Dish.txt", ios::app); // 打开文件以追加方式写入
        if (!dishFile.is_open())
        {
            cout << "Error opening file!" << endl;
            return;
        }

        Dish newDish(dish_name, dish_id, dish_price, dish_ingredients, dish_cost);
        newDish.writeToFile(dishFile); // 将菜品信息写入文件
        cout << "New Dish Added Successfully!" << endl;

        dishFile.close(); // 关闭文件
    }

    void deleteDish(int dish_id)
    {
        vector<Dish> dishes = readDishesFromFile("Dish.txt");
        bool found = false;
        ofstream tempFile("temp.txt");
        for (const auto& dish : dishes)
        {
            if (dish.getId() == dish_id)
            {
                found = true;
                cout << "Dish deleted successfully:" << endl;
            }
            else
            {
                dish.writeToFile(tempFile); // 将非被删除的菜品写入临时文件
            }
        }
        tempFile.close();

        if (!found)
        {
            cout << "Dish not found!" << endl;
            remove("temp.txt"); // 删除临时文件
            return;
        }

        // 删除原文件并重命名临时文件
        remove("Dish.txt");
        rename("temp.txt", "Dish.txt");
    }
};

void updateMaxOrderIDFile(int newMaxOrderID)
{
    ofstream file("MaxOrderID.txt");
    if (file.is_open())
    {
        file << newMaxOrderID; // 写入新的最大OrderID
        file.close();
    }
    else
    {
        cerr << "Unable to open MaxOrderID.txt for writing." << endl;
    }
}

int getMaxOrderID()
{
    ifstream file("MaxOrderID.txt");
    int maxOrderID = 0; // 默认OrderID从1开始，如果文件不存在或为空
    if (file.is_open())
    {
        file >> maxOrderID; // 读取最大OrderID
        file.close();
    }
    return maxOrderID;
}

class Customer : public User
{
private:
    vector<Order> orders;

public:
    Customer(string uname, string mail, int user_age) : User(uname, mail, user_age)
    {
        if (!userInfoExists(userinfo_filename))
        {
            ofstream ofile(userinfo_filename, ios_base::app);
            if (!ofile)
            {
                cerr << "Error opening file." << endl;
            }
            else
            {
                writeToFile(ofile);
            }
        }
        loadOrdersFromFile("orders.txt");
    }

    void loadOrdersFromFile(const string& filename)
    {
        ifstream file(filename);
        if (!file.is_open())
        {
            cerr << "Unable to open the file: " << filename << endl;
            return;
        }

        string line, key, value, email, time_;
        int orderID, year, month, day, dishID;
        double price, cost;
        bool paid;

        while (getline(file, line))
        {
            if (line.empty())
                continue; // Skip empty lines

            size_t colonPos = line.find(':');
            if (colonPos != string::npos)
            {
                key = line.substr(0, colonPos);
                value = line.substr(colonPos + 1);
                if (!value.empty() && value.front() == ' ')
                {
                    value.erase(0, 1); // Remove leading space
                }
                if (!value.empty() && value.front() == ' ')
                    value.erase(0, 1); // Remove leading space

                if (key == "Email")
                    email = value;
                else if (key == "ID")
                    orderID = stoi(value);
                else if (key == "Year")
                    year = stoi(value);
                else if (key == "Month")
                    month = stoi(value);
                else if (key == "Day")
                    day = stoi(value);
                else if (key == "Time")
                    time_ = value;
                else if (key == "Dish ID")
                    dishID = stoi(value);
                else if (key == "Price")
                    price = stod(value);
                else if (key == "Cost")
                    cost = stod(value);
                else if (key == "Paid")
                    paid = stoi(value);

                if (file.peek() == '\n' || file.peek() == EOF)
                { // Check for end of order
                    if (this->email == email)
                    { // Only load orders for the specified customer
                        orders.push_back(Order(orderID, email, year, month, day, time_, dishID, price, cost, paid));
                    }
                }
            }
        }
    }

    void getUserInfo() override
    {
        cout << "Customer Information:" << endl;
        User::getUserInfo();
    }

    void editUserInfo() override
    { // 是否保留
        cout << "Editing Customer Information:" << endl;
        User::editUserInfo();
    }
    void writeToFile(ofstream& userinfo_filename) override
    {
        userinfo_filename << "Usertype: " << "Customer" << endl;
        userinfo_filename << "Username: " << username << endl;
        userinfo_filename << "Email: " << email << endl;
        userinfo_filename << "Age: " << age << endl;
        userinfo_filename << endl;
    }

    void orderdish(const string& filename, int dish_ID)
    {
        vector<Dish> dishes = readDishesFromFile(filename);
        // 遍历 dishes 向量中的每个 Dish 对象
        for (const Dish& dish : dishes)
        {
            // 检查当前 Dish 对象的 ID 是否与目标 ID 相匹配
            if (dish.getId() == dish_ID)
            {

                time_t curtime = time(nullptr);
                tm* nowtime = localtime(&curtime);
                if (nowtime == nullptr)
                {
                    cerr << "Local time could not be retrieved." << endl;
                    return;
                }
                int currentMaxOrderID = getMaxOrderID();
                int newOrderID = currentMaxOrderID + 1; // 生成新的OrderID
                orders.push_back(Order(newOrderID, email, 1900 + nowtime->tm_year, 1 + nowtime->tm_mon, nowtime->tm_mday, to_string(nowtime->tm_hour) + ':' + to_string(nowtime->tm_min) + ':' + to_string(nowtime->tm_sec), dish_ID, dish.getPrice(), dish.getCost(), false));
                updateMaxOrderIDFile(newOrderID); // 更新最大OrderID文件
                cout << "Dish ordered successfully." << endl;
                return;
            }
        }
        cout << "Dish not found" << endl;
    }

    void showOrders(bool showOnlyNotPaid)
    {
        cout << "Displaying " << (showOnlyNotPaid ? "unpaid" : "all") << " orders:" << endl;
        for (const Order& order : orders)
        {
            if (showOnlyNotPaid && order.isPaid())
            {
                continue; // 如果只显示未支付订单，跳过已支付的订单
            }
            order.displayOrderDetails();
            cout << endl; // 在订单之间添加空行以提高可读性
        }
    }

    void pay()
    {
        double totalDue = 0;
        vector<Order*> unpaidOrders;

        // Calculate total due for unpaid orders
        for (Order& order : orders)
        {
            if (!order.isPaid())
            {
                totalDue += order.getPrice();
                unpaidOrders.push_back(&order);
            }
        }

        cout << "Total amount due: $" << totalDue << endl;
        if (totalDue == 0)
        {
            cout << "No outstanding payments." << endl;
            return;
        }

        cout << "Do you want to proceed with payment? (Y/N): ";
        char confirmation;
        cin >> confirmation;

        if (confirmation == 'Y' || confirmation == 'y')
        {
            for (Order* order : unpaidOrders)
            {
                // First delete the existing order from the file
                order->deleteOrderFromFile("orders.txt");

                // Update the order to paid
                order->setPaid(true);

                // Write the updated order back to the file
                order->writeToFile("orders.txt");
            }
            cout << "Payment successful." << endl;
        }
        else
        {
            cout << "Payment cancelled." << endl;
        }
    }
};

void printCustomerMenu()
{
    cout << "\nMenu:" << endl;
    cout << "1. show dish menu" << endl;
    cout << "2. order" << endl;
    cout << "3. display orders not paid" << endl;
    cout << "4. display all orders" << endl;
    cout << "5. pay" << endl; // 添加新选项
    cout << "6. Exit" << endl;
    cout << "Enter your choice: ";
}

void customerMenu(User* user)
{
    int menuChoice;
    do
    {
        printCustomerMenu();
        cin >> menuChoice;
        switch (menuChoice)
        {
        case 1:
        {
            user->print_dish_menu("Dish.txt");
            break;
        }
        case 2:
        {
            int dishId;
            cout << "Enter Dish ID to order: ";
            cin >> dishId;
            dynamic_cast<Customer*>(user)->orderdish("Dish.txt", dishId);
            break;
        }
        case 3:
        {
            dynamic_cast<Customer*>(user)->showOrders(1);
            break;
        }
        case 4:
        {
            dynamic_cast<Customer*>(user)->showOrders(0);
            break;
        }
        case 5:
        {
            dynamic_cast<Customer*>(user)->pay();
            break;
        }
        case 6:
        {
            cout << "Exiting Customer UI..." << endl;
            break;
        }
        default:
            cout << "Invalid choice. Please try again." << endl;
        }
    } while (menuChoice != 6);
}

void customer_logging(User* user)
{
    string input_name;
    string customor_email;
    int age = 0;
    cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard bad input
    while (true)
    {
        cout << "Please enter your name: ";
        getline(cin, input_name);

        // Check if the name is empty
        if (input_name.empty())
        {
            cout << "Please enter a valid name.\n";
            continue; // Prompt the user to enter a name again
        }

        // Check if the name contains only alphabets and spaces
        bool valid = true;
        for (char c : input_name)
        {
            if (!isalpha(c) && !isspace(c))
            {
                valid = false;
                break;
            }
        }

        if (!valid)
        {
            cout << "Name should contain only alphabets and spaces.\n";
            continue; // Prompt the user to enter a name again
        }

        // If the name is valid, break the loop
        break;
    }
    cout << "Enter Your Email: ";
    while (!(cin >> customor_email) || (customor_email.find('@')))
    {
        cin.clear(); // Clear the error flag
        cerr << "Invalid input. Please enter a right email." << endl;
        cout << "Enter Your Email: ";
        cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard bad input
    }
    cout << "Enter Your age: ";
    while (!(cin >> age) || age <= 0)
    {
        cin.clear(); // Clear the error flag
        cerr << "Invalid input. Please enter a right age." << endl;
        cout << "Enter Your age: ";
        cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Discard bad input
    }
    user = new Customer(input_name, customor_email, age); // from here start
    customerMenu(user);
    system("cls");
}

void printChefMenu()
{
    cout << "\nMenu:" << endl;
    cout << "1. Edit User Info" << endl;
    cout << "2. Delete Dish" << endl;
    cout << "3. Add New Dish" << endl; // 添加新选项
    cout << "4. Exit" << endl;
    cout << "Enter your choice: ";
}

void ChefMenu(User* user)
{
    int menuChoice;
    do
    {
        printChefMenu();
        cin >> menuChoice;
        switch (menuChoice)
        {
        case 1:
            user->editUserInfo(); // 修改用户信息
            cout << "Updated User Info:" << endl;
            user->getUserInfo(); // 打印更新后的用户信息
            break;
        case 2:
        {
            int dishId;
            cout << "Enter Dish ID to delete: ";
            cin >> dishId;
            dynamic_cast<Chef*>(user)->deleteDish(dishId); // 删除菜品
            break;
        }
        case 3:
            int newDishId;
            cout << "Enter Dish ID for the new dish: ";
            cin >> newDishId;
            dynamic_cast<Chef*>(user)->addNewDish(newDishId); // 添加菜品
            break;
        case 4:
            cout << "Exiting program..." << endl;
            break;
        default:
            cout << "Invalid choice. Please try again." << endl;
        }
    } while (menuChoice != 4);
}

void ManagerPrintMenu()
{
    cout << "\nManager Menu:" << endl;
    cout << "1. View All Orders" << endl;
    cout << "2. View Chef Info" << endl;
    cout << "3. View Customer Info" << endl;
    cout << "4. Modify Chef" << endl;
    cout << "5. Delete Dish" << endl;
    cout << "6. Add dish" << endl;
    cout << "7. Calculate Revenue" << endl;
    cout << "8. Sell Dish" << endl; // 新增销售菜品的选项
    cout << "9. Exit" << endl;
    cout << "Enter your choice: ";
}

void ManagerMenu(Manager* manager)
{
    int ManagerMenuChoice;
    do
    {
        ManagerPrintMenu();
        cin >> ManagerMenuChoice;
        switch (ManagerMenuChoice)
        {
        case 1:
            manager->viewAllOrders();
            break;
        case 2:
            manager->viewChefInfo();
            break;
        case 3:
            manager->viewCustomerInfo();
            break;
        case 4:
        {
            int chefId;
            cout << "Enter Chef ID to modify: ";
            cin >> chefId;
            manager->modifyChef(chefId);
            break;
        }
        case 5:
        {
            int dishId;
            cout << "Enter Dish ID to delete: ";
            cin >> dishId;
            manager->deleteDish(dishId); // 假设Manager类有deleteDish方法
            break;
        }
        case 6:
        {
            // cout << "Adding New Dish:" << endl;
            // string dishName, ingredients;
            // double price, cost;
            // int newDishId;

            // cout << "Enter Dish Name: ";
            // cin.ignore(); // 忽略之前的换行符
            // getline(cin, dishName);
            // cout << "Enter Price: ";
            // cin >> price;
            // cout << "Enter Ingredients: ";
            // cin.ignore(); // 忽略之前的换行符
            // getline(cin, ingredients);
            // cout << "Enter Cost: ";
            // cin >> cost;

            manager->addNewDish(); // 假设Manager类有addDish方法
            break;
        }

        case 7:
            manager->calculateRevenue(); // 调用计算收入的功能
            break;
        case 8:
        {
            int dishId, quantity;
            cout << "Enter Dish ID to sell: ";
            cin >> dishId;
            cout << "Enter quantity sold: ";
            cin >> quantity;
            manager->sellDish(dishId, quantity);
            break;
        }
        // 其他case处理保持不变
        case 9:
            cout << "Exiting program..." << endl;
            return;
        default:
            cout << "Invalid choice. Please try again." << endl;
            break;
        }
    } while (ManagerMenuChoice != 9);
}
bool authenticate(User* user)
{
    // 只有Manager和Chef需要密码验证
    if (dynamic_cast<Manager*>(user) || dynamic_cast<Chef*>(user))
    {
        string correctPassword = user->getPassword();
        string inputPassword;
        int attempts = 0;
        while (attempts < 3)
        {
            cout << "Enter your password: ";
            cin >> inputPassword;
            if (inputPassword == correctPassword)
            {
                return true;
            }
            else
            {
                attempts++;
                cout << "Incorrect password. Please try again." << endl;
            }
        }
        cout << "Too many incorrect attempts. Exiting..." << endl;
        return false;
    }
    return true; // Customer不需要密码验证
}

void chef_logging(User* user)
{
    user = new Chef("chef123", "654321", "chef@example.com", 30);
    if (authenticate(user))
    {
        cout << "Access granted." << endl;
        user->editUserInfo(); // 用户输入个人信息
        cout << "Your User Info:" << endl;
        user->getUserInfo(); // 打印用户信息

        // 检查用户信息是否已存在
        if (!user->userInfoExists("userinfo.txt"))
        {
            ofstream outFile("userinfo.txt", ios::app);
            if (!outFile)
            {
                cerr << "Error opening file." << endl;
                delete user;
            }
            user->writeToFile(outFile); // 将用户信息写入文件
            outFile.close();
            cout << "User Info added to file." << endl;
        }
        else
        {
            cout << "User Info already exists in file." << endl;
        }
        ChefMenu(user); // 处理菜单
    }
    else
    {
        cout << "Access denied." << endl;
    }
}

void managerloger(User* user)
{
    user = new Manager("manager123", "123456", "manager@example.com", 35);
    if (authenticate(user))
    {
        cout << "Access granted." << endl;
        user->editUserInfo();
        cout << "Your User Info:" << endl;
        user->getUserInfo();

        if (Manager* manager = dynamic_cast<Manager*>(user))
        {
            ManagerMenu(manager); // 调用经理菜单
        }
        else if (Chef* chef = dynamic_cast<Chef*>(user))
        {
            ChefprintMenu(); // 假设存在一个专门为厨师定义的菜单
        }
        else if (Customer* customer = dynamic_cast<Customer*>(user))
        {
            // 处理顾客菜单或操作
        }
    }
    else
    {
        cout << "Access denied." << endl;
    }
}

int main()
{
    int choice;
    cout << "Select your role:" << endl;
    cout << "1. Manager" << endl;
    cout << "2. Chef" << endl;
    cout << "3. Customer" << endl;
    cout << "Enter your choice: ";
    cin >> choice;

    User* user = nullptr;

    switch (choice)
    {
    case 1:
        managerloger(user);
        break;
    case 2:
        chef_logging(user);
        break;
    case 3:
    {
        customer_logging(user);
        break;
    }
    default:
        cout << "Invalid choice. Exiting..." << endl;
        return 0;
    }

    delete user; // 释放内存

    return 0;
}
