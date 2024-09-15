/// <reference path="C:\Program Files\Leica Geosystems\Cyclone 3DR 19.1\Script\JsDoc\Reshaper.d.ts"/>

// ------------------------ HOW TO USE IT --------------------------------------------
// This algoritm is a modification of the TreeMeshing.js. The algorithm uses the selected clouds to mesh them using this trunk meshing strategy! => Make sure to select the clouds firts
//
// ------------------------ ALGORITHM USED --------------------------------------------
// 1. The algorithm creates slices of defined depth in the point cloud at regular steps in Z
// 2. A section of each slice is created using the convex hull algorithm
// 3. The sections are joined in order to create a watertight mesh
// 4. MODIFICATION - it also calculates the total volume
//
// ------------------------ PARAMETERS --------------------------------------------
// Parameter explanations:
// 1. Slice step: distance between the sections that have to be created
// 2. Slice depth: depth of each section for creating the convex hull
// 


function ErrorMessage(_iMessage // [in] the error message
    , _iThrowError = true // [in] should we throw an error (default value: true)
) {
    var _iThrowError = (typeof _iThrowError !== 'undefined') ? _iThrowError : true;

    var myDlg = SDialog.New("Error Message");
    myDlg.AddLine(_iMessage, false, {}, 1);
    myDlg.Execute();
    if (_iThrowError)
        throw new Error(_iMessage);
}


function mainTreeMeshing() {
    // Total Volume
    var VolumeTotal = 0
    // retrieving the cloud to mesh with this approach
    var selClouds = SCloud.FromSel();
    if (selClouds.length == 0)
       ErrorMessage("Esqueceu de selecionar as nuvens de pontos!")
    for (let i = 0; i<selClouds.length; i++) {
        var theCloud = selClouds[i];
    
    

        //Enter the input data
        //var theDialog = SDialog.New('Tree meshing parameters');
        //theDialog.AddLine("Slice step: ", true, {}, 1);
        //theDialog.AddLine("Slice depth: ", true, {}, 0.5);

        //var result = theDialog.Execute();
        //if (result.ErrorCode != 0)// result == 0 means the user click //on the "OK" button
            //ErrorMessage("Operation canceled");

        var Param_Slice_Step = 1 //parseFloat(result.InputTbl[0]);
        var Param_Slice_Depth = 0.5 //parseFloat(result.InputTbl[1]);

        // looking for the different heights at which we will create the mesh
        var ZVect = SVector.New(0, 0, 1);
        var centroid = theCloud.GetCentroid().Point;
        var LPt = theCloud.GetLowestPoint(ZVect).Point.GetZ();
        var UPt = theCloud.GetHighestPoint(SVector.New(0, 0, 1)).Point.GetZ();
        try{
            if (UPt - LPt < 2 * Param_Slice_Step)
                console.error('Too few steps will be created to mesh')
                //ErrorMessage("Too few steps will be created to mesh")
        } catch (error) {
            //console.error('An error occurred:', error);
            continue;
        }

        var NStep = Math.round((UPt - LPt) / Param_Slice_Step);

        // looping on each height to create the convex contours
        var allMultis = new Array;
        for (var iHeight = 0; iHeight < NStep; iHeight++) {
            var curZ = LPt + iHeight * Param_Slice_Step;
            var locPoint = SPoint.New(centroid.GetX(), centroid.GetY(), curZ);
            var curPlane = SPlane.New(locPoint, ZVect, SVector.New(1, 0, 0), 1, 1);

            var localCloud = theCloud.SeparateFeature(curPlane, Param_Slice_Depth, SCloud.FILL_IN_ONLY).InCloud;

            try{
            var convContRes = localCloud.GetConvexContour(ZVect, locPoint, false);
            
            if (convContRes.ErrorCode != 0)
                console.error("Error when extracting convex contours")
            } catch (error) {
                continue;
                //ErrorMessage("Error when extracting convex contours")
            }
            allMultis.push(convContRes.Multi)
        }

        // creating the meshes at each heights

        var allPolys = new Array;
        // adding bottom
        try{
        var bottomRes = SPoly.ConstraintMesh2D(null, [allMultis[0]], ZVect, 0, 0);
        
        if (bottomRes.ErrorCode != 0 || bottomRes.PolyTbl.length != 1)
                console.error('Error when creating bottom')
                //ErrorMessage("Too few steps will be created to mesh")
        } catch (error) {
            //console.error('An error occurred:', error);
            continue;
        }
            //ErrorMessage("Error when creating bottom")
        allPolys.push(bottomRes.PolyTbl[0])

        // looping on each height
        for (var iHeight = 0; iHeight < NStep - 1; iHeight++) {
            var locLMultis = new Array;
            locLMultis.push(allMultis[iHeight])
            locLMultis.push(allMultis[iHeight + 1])
            
            try {

            var meshRes = SPoly.JoinContour(locLMultis, [], 0, 0, 0, false, 0);
            
            if (meshRes.ErrorCode != 0)
                console.error("Error when joining contours")
            } catch (error) {
                continue;
                //ErrorMessage("Error when joining contours")
            }
            allPolys.push(meshRes.Poly)
        }

        // adding top
        try {
        
        var topRes = SPoly.ConstraintMesh2D(null, [allMultis[allMultis.length - 1]], ZVect, 0, 0);
        
        if (topRes.ErrorCode != 0 || topRes.PolyTbl.length != 1)
            console.error("Error when creating top")
        } catch (error) {
            //ErrorMessage("Error when creating top")
            continue;
        }
        allPolys.push(topRes.PolyTbl[0])

        // merging the results (final should be only 1 filled mesh)
        try {
        
        var resMerge = SPoly.MergeCommonBorders(allPolys, SPoly.SIMPLE);
        
        if (resMerge.ErrorCode != 0)
            console.error("Error when mergin meshes")
        } catch (error) {
            continue;
            //ErrorMessage("Error when mergin meshes")
        }

        var finalMeshes = resMerge.PolyTbl;
        for (var iMesh = 0; iMesh < finalMeshes.length; iMesh++) {
            finalMeshes[iMesh].SetName(theCloud.GetName())
            finalMeshes[iMesh].AddToDoc();
        }
        //print())
       VolumeTotal = VolumeTotal + finalMeshes[0].GetVolume().Volume; 

        console.log("Done", i)

    }
    print((`O Volume Total é:  ${VolumeTotal} unidades de volume`))
}

mainTreeMeshing()
