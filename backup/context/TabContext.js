import React, { createContext, useContext, useReducer } from 'react';

// Create the context
const TabContext = createContext();

// Initial state for the tab system
const initialState = {
  activeTab: 'home',
  modules: [
    { id: 'home', label: 'Home', visible: true },
    { id: 'module1', label: 'Module 1', visible: false },
    { id: 'module2', label: 'Module 2', visible: false },
    { id: 'module3', label: 'Module 3', visible: false },
    { id: 'module4', label: 'Module 4', visible: false }
  ]
};

// Reducer function to handle tab state changes
const tabReducer = (state, action) => {
  switch (action.type) {
    case 'SET_ACTIVE_TAB':
      // Update the active tab and adjust visibility of modules
      const updatedModules = state.modules.map(module => ({
        ...module,
        visible: module.id === action.payload
      }));

      return {
        ...state,
        activeTab: action.payload,
        modules: updatedModules
      };

    case 'RESET_TO_HOME':
      // Reset to home view showing all modules
      const homeModules = state.modules.map(module => ({
        ...module,
        visible: module.id === 'home'
      }));

      return {
        ...state,
        activeTab: 'home',
        modules: homeModules
      };

    default:
      return state;
  }
};

// Provider component
export const TabProvider = ({ children }) => {
  const [state, dispatch] = useReducer(tabReducer, initialState);

  // Function to switch to a specific module tab
  const switchToModule = (moduleId) => {
    dispatch({ type: 'SET_ACTIVE_TAB', payload: moduleId });
  };

  // Function to switch to home view
  const switchToHome = () => {
    dispatch({ type: 'RESET_TO_HOME' });
  };

  return (
    <TabContext.Provider value={{
      state,
      switchToModule,
      switchToHome
    }}>
      {children}
    </TabContext.Provider>
  );
};

// Custom hook to use the tab context
export const useTabContext = () => {
  const context = useContext(TabContext);
  if (!context) {
    throw new Error('useTabContext must be used within a TabProvider');
  }
  return context;
};

export default TabContext;