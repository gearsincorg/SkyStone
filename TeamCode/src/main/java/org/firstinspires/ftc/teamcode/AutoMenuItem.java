package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;
import java.util.List;

public class AutoMenuItem
{
    private String menuName;
    private FieldLocation selection;
    private List<FieldLocation> options = null;


    public AutoMenuItem (String name, boolean isStartMenu)
    {
        menuName = name;
        selection = FieldLocation.NONE;
        if (isStartMenu)
            preloadStartLocations();
        else
            preloadDestinations();
    }

    public void preloadStartLocations () {
        options = new LinkedList<>();
        options.add(FieldLocation.NONE);
        options.add(FieldLocation.S_RAMP);
        options.add(FieldLocation.S_SAFE);
        options.add(FieldLocation.S_COLLECT);
    }

    public void preloadDestinations () {
        options = new LinkedList<>();
        options.add(FieldLocation.NONE);
        options.add(FieldLocation.D_NEAR_BEACON);
        options.add(FieldLocation.D_FAR_BEACON);
        options.add(FieldLocation.D_SHOOT);
        options.add(FieldLocation.D_BALL);
        options.add(FieldLocation.D_CENTER);
        options.add(FieldLocation.D_RAMP);
        options.add(FieldLocation.D_AWAY);
    }

    public FieldLocation getSelection() {
        return selection;
    }

    public String getName() {
        return menuName;
    }

    public FieldLocation setSelection(FieldLocation newSelection) {
        // Make sure new selection is available in list
        if (findIndex(newSelection) >= 0) {
            selection = newSelection;
        }
        else
            selection = FieldLocation.NONE;

        return selection;
    }

    public void next() {
        // find the current selection in the list and then get the next one
        int index;
        if ((index = findIndex(selection)) >= 0) {
            if (index < (options.size() - 1))
                selection = options.get(index+1);
        }
        else
            selection = FieldLocation.NONE;
    }

    public void prev()
    {
        int index;
        if ((index = findIndex(selection)) >= 0)
        {
            if (index > 0)
                selection = options.get(index -1);
        }
        else
            selection = FieldLocation.NONE;
    }

    public int findIndex(FieldLocation loc) {
        int index = -1;
        for (int i = 0; i < options.size(); i++) {
            if (options.get(i).equals(loc)) {
                index = i;
                break;
            }
        }
        return index;
    }
}