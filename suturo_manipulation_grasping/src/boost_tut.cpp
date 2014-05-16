#include <boost/thread.hpp>
#include <iostream>

using namespace std;

bool moveSuccess = false;
bool collision   = false;

void ThreadFunction()
{
    int counter = 0;

    for(int i=0; i<=5; ++i)
    {
        cout << "thread iteration " << ++counter << " Press Enter to stop" << endl;

        try
        {
            // Sleep and check for interrupt.
            // To check for interrupt without sleep,
            // use boost::this_thread::interruption_point()
            // which also throws boost::thread_interrupted
            //
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
        catch(boost::thread_interrupted&)
        {
            cout << "Thread is stopped" << endl;
            moveSuccess = true;
            return;
        }
    }
    moveSuccess = true;
}

int main()
{
    // Start thread
    boost::thread t(&ThreadFunction);
    
    while(!moveSuccess)
    { 
        cout << "checking collision" << endl;
        // if (collision_found) move_group.stop() bla bla;
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    // Join - wait when thread actually exits
    t.join();
    cout << "main: thread ended" << endl;

    return 0;
}
