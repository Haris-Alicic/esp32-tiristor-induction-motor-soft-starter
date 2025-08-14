#ifndef MULTITASK_HPP
#define MULTITASK_HPP

#include <Arduino.h>

class Flag;

class FlagPole
{
    private:
    uint8_t update_counter = 0;
    SemaphoreHandle_t _semaphore;
    friend class Flag;

    uint8_t get()
    {
        return update_counter;
    }

    void set()
    {
        ++update_counter;
    }

    SemaphoreHandle_t& getMutex()
    {
        return _semaphore;
    }

    public:
    FlagPole()
    {
        _semaphore = xSemaphoreCreateMutex();
    }
};

class Flag
{
    private:
    FlagPole& _flagPole;
    SemaphoreHandle_t _semaphore;
    uint8_t _prevoious_update_counter = 0;
    public:
    Flag(FlagPole &setupPole): _flagPole(setupPole)
    {
        _semaphore = _flagPole.getMutex();
    }

    void set()
    {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
        _flagPole.set();
        xSemaphoreGive(_semaphore);
    }

    bool get()
    {
        bool result = false;
        xSemaphoreTake(_semaphore, portMAX_DELAY);
        if(_prevoious_update_counter != _flagPole.get())
        {
            _prevoious_update_counter = _flagPole.get();
            result = true;
        }
        xSemaphoreGive(_semaphore);
        return result;
    }
};

/*************************************************************************************************/

template <typename T> class DataReader;
template <typename T> class DataWriter;

template <typename T>
class DataStore
{
    friend class DataReader<T>;
    friend class DataWriter<T>;
    private:
    T _data;
    SemaphoreHandle_t _semaphore;

    public:
    DataStore()
    {
        _semaphore = xSemaphoreCreateMutex(); 
    }
};

template <typename T>
class DataReader
{
    private:
    DataStore<T>& _dataStore;
    SemaphoreHandle_t _semaphore;
    public:
    DataReader(DataStore<T>& dataStore): _dataStore(dataStore)
    {
        _semaphore = _dataStore._semaphore;
    }

    T read()
    {
        return _dataStore._data;
    }
};

template <typename T>
class DataWriter
{
    private:
    DataStore<T>& _dataStore;
    SemaphoreHandle_t _semaphore;
    public:
    DataWriter(DataStore<T>& dataStore): _dataStore(dataStore)
    {
        _semaphore = _dataStore._semaphore;
    }
    
    void write(T data)
    {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
        _dataStore._data = data;
        xSemaphoreGive(_semaphore);
    }
};

#endif //MULTITASK_HPP